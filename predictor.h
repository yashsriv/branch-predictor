/* Author: Jared Stark;   Created: Tue Jul 27 13:39:15 PDT 2004
 * Description: This file defines a gshare branch predictor.
 */

#ifndef PREDICTOR_H_SEEN
#define PREDICTOR_H_SEEN

#include <cstddef>
#include <inttypes.h>
#include <map>
#include <vector>
#include "op_state.h"   // defines op_state_c (architectural state) class
#include "tread.h"      // defines branch_record_c class

#define abs(x) ((x)<0 ? -(x) : (x))


class PREDICTOR
{
public:
  typedef uint32_t address_t;

private:
  typedef uint64_t path_t;
  typedef unsigned __int128 history_t;
  typedef int32_t counter_t;

  static const int NUM_TABLES = 8;
  const int L[NUM_TABLES] = {0, 2, 4, 8, 16, 32, 64, 128};
  const size_t PHT_SIZES[NUM_TABLES] = {11, 11, 11, 11, 11, 11, 11, 11};
  const int COUNTER_BITS[NUM_TABLES] = {5, 5, 4, 4, 4, 4, 3, 3};

  // Path History
  static const int PATH_HIST_LENGTH = 64;  // 64 bits
  static const path_t PATH_HIST_MASK = ((unsigned __int128)(1) << PATH_HIST_LENGTH) - 1;

  // Global History
  static const int GLOBAL_HIST_LENGTH = (1 << (NUM_TABLES - 1));  // 128 bits

  static const counter_t PHT_INIT = /* very weakly taken */ 0;

  history_t ghist;                         // 128 bits
  path_t phist;                            // 64 bits
  std::vector<std::vector<counter_t>> pht; // 2 x 2K x 5 bits + 4 x 2K x 4 bits + 2 x 2K x 3 bits = 64K
  counter_t THRESH = NUM_TABLES;
  counter_t TC = 0;

  void update_ghist(bool taken) {
    ghist <<= 1;
    if (taken)
      ghist |= 1;
  }

  void update_phist(path_t addr_bit) {
    phist <<= 1;
    phist &= PATH_HIST_MASK;
    phist |= addr_bit;
  }
  static counter_t counter_inc(/* n-bit counter */ counter_t cnt, int bits) {
    if (cnt != (1 << (bits - 1)) - 1)
      ++cnt;
    return cnt;
  }
  static counter_t counter_dec(/* n-bit counter */ counter_t cnt, int bits) {
    if (cnt != -(1 << (bits - 1)))
      --cnt;
    return cnt;
  }

public:
  PREDICTOR(void)
    : ghist(0)
    , phist(0)
    , pht(NUM_TABLES)
  {
    for (std::size_t it = 0; it < NUM_TABLES; ++it) {
      pht[it] = std::vector<counter_t>(std::size_t(1) << PHT_SIZES[it], counter_t(PHT_INIT));
    }
  }
  // uses compiler generated copy constructor
  // uses compiler generated destructor
  // uses compiler generated assignment operator

  std::vector<std::size_t> get_indices(address_t pc) {
    std::vector<std::size_t> indices(NUM_TABLES, -1);
    for (int i = 0; i < NUM_TABLES; ++i) {
      std::size_t index;
      std::size_t PHT_INDEX_MASK = (std::size_t(1) << PHT_SIZES[i]) - 1;
      if (L[i] == 0) {
        index = pc & PHT_INDEX_MASK;
      } else {
        history_t bitvector = 0;
        history_t GLOBAL_HIST_MASK = (1 << L[i]) - 1;
        history_t ghist_bits = ghist & GLOBAL_HIST_MASK;
        std::size_t bitsfilled = 0;
        // First add path history
        if (L[i] < PATH_HIST_LENGTH) {
          bitvector |= phist & GLOBAL_HIST_MASK;
          bitsfilled += L[i];
        } else {
          bitvector |= phist;
          bitsfilled += PATH_HIST_LENGTH;
        }
        // Then global branch history
        ghist_bits <<= bitsfilled;
        bitvector |= ghist_bits;
        bitsfilled += L[i];
        // Then PC
        bitvector |= (history_t(pc) << bitsfilled);

        index = 0; // XOR Identity
        for (int j = PHT_SIZES[i]; j < 128; j += PHT_SIZES[i]) {
          index ^= bitvector & PHT_INDEX_MASK;
          bitvector >>= PHT_SIZES[i];
        }
      }
      indices[i] = index;
    }
    return indices;
  }

  // get_prediction() takes a branch record (br, branch_record_c is defined in
  // tread.h) and architectural state (os, op_state_c is defined op_state.h).
  // Your predictor should use this information to figure out what prediction it
  // wants to make.  Keep in mind you're only obligated to make predictions for
  // conditional branches.
  bool get_prediction(const branch_record_c* br, const op_state_c*) {
    bool prediction = false;
    if (/* conditional branch */ br->is_conditional) {
      address_t pc = br->instruction_addr;

      counter_t sum = calc_sum(pc);

      prediction = sum >= 0;
    }
    return prediction;   // true for taken, false for not taken
  }

  counter_t calc_sum(address_t pc) {
    counter_t sum = NUM_TABLES / 2;

    std::vector<std::size_t> indices = get_indices(pc);

    for (int i = 0; i < NUM_TABLES; ++i) {
      sum += pht[i][indices[i]];
    }

    return sum;
  }

  void update_counters(address_t pc, bool taken) {
    std::vector<std::size_t> indices = get_indices(pc);
    for (int i = 0; i < NUM_TABLES; ++i) {
      std::size_t index = indices[i];
      counter_t cnt = pht[i][index];
      if (taken)
        cnt = counter_inc(cnt, COUNTER_BITS[i]);
      else
        cnt = counter_dec(cnt, COUNTER_BITS[i]);
      pht[i][index] = cnt;
    }
  }

  // Update the predictor after a prediction has been made.  This should accept
  // the branch record (br) and architectural state (os), as well as a third
  // argument (taken) indicating whether or not the branch was taken.
  void update_predictor(const branch_record_c* br, const op_state_c* os, bool taken) {

    address_t pc =  br->instruction_addr;
    if (/* conditional branch */ br->is_conditional) {
      bool pred = get_prediction(br, os);
      counter_t sum = calc_sum(pc);

      if (pred != taken || abs(sum) < THRESH) {
        update_counters(pc, taken);
      }

      // Dynamic Thresholding
      if (pred != taken) {
        ++TC;
        if (TC == 63) {
          ++THRESH;
          TC = 0;
        }
      }
      if ((pred == taken) && (abs(sum) < THRESH)) {
        --TC;
        if (TC == -64) {
          --THRESH;
          TC = 0;
        }
      }
      update_ghist(taken);
      update_phist(pc % 2);
    } else if (br->is_call || br->is_return || br->is_indirect) {
      update_ghist(true);
      update_phist(pc % 2);
    }

  }
};

#endif // PREDICTOR_H_SEEN

