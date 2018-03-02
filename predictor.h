/* Author: Jared Stark;   Created: Tue Jul 27 13:39:15 PDT 2004
 * Description: This file defines a gshare branch predictor.
 */

#ifndef PREDICTOR_H_SEEN
#define PREDICTOR_H_SEEN

#include <bitset>
#include <cstddef>
#include <inttypes.h>
#include <map>
#include <vector>
#include "op_state.h"   // defines op_state_c (architectural state) class
#include "tread.h"      // defines branch_record_c class

#define abs(x) ((x)<0 ? -(x) : (x))


class PREDICTOR {
public:
  typedef uint32_t address_t;

private:
  typedef uint32_t path_t;
  typedef unsigned __int128 history_t;
  typedef int8_t counter_t;

  static const int NUM_TABLES = 8;
  const size_t L[NUM_TABLES] = {0, 2, 4, 8, 16, 32, 64, 128};
  const size_t PHT_SIZES[NUM_TABLES] = {11, 11, 11, 11, 11, 11, 11, 11};
  const size_t COUNTER_BITS[NUM_TABLES] = {5, 5, 4, 4, 4, 4, 3, 3};

  // Path History
  static const int PATH_HIST_LENGTH = 32;  // 32 bits
  static const path_t PATH_HIST_MASK = ((unsigned __int128)(1) << PATH_HIST_LENGTH) - 1;

  // Global History
  static const int GLOBAL_HIST_LENGTH = (1 << (NUM_TABLES - 1));  // 128 bits

  static const counter_t PHT_INIT = /* very weakly taken */ 0;

  history_t ghist;                         // 128 bits
  path_t phist;                            // 32 bits

  std::size_t indices[NUM_TABLES];        // Indices to the pht
  double sum;
  std::vector<counter_t> pht[NUM_TABLES]; // 2 x 2K x 5 + 4 x 2K x 4 + 3 x 2K x 2 = 64K

  counter_t THRESH;
  counter_t TC;

  void update_ghist(bool taken) {
    ghist <<= 1;
    if (taken)
      ghist |= history_t(1);

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
    , THRESH(NUM_TABLES)
    , TC(0)
  {
    for (std::size_t it = 0; it < NUM_TABLES; ++it) {
      pht[it] = std::vector<counter_t>(std::size_t(1) << PHT_SIZES[it], counter_t(PHT_INIT));
    }
  }
  // uses compiler generated copy constructor
  // uses compiler generated destructor
  // uses compiler generated assignment operator

  void calc_indices(address_t pc) {
    for (int i = 0; i < NUM_TABLES; ++i) {
      std::size_t PHT_INDEX_MASK = (std::size_t(1) << PHT_SIZES[i]) - 1;
      std::size_t index = pc & PHT_INDEX_MASK;
      if (L[i] != 0) {
        typedef std::bitset<128 + PATH_HIST_LENGTH + 32> index_t;
        index_t bitvector;
        // history_t bitvector = 0;
        index_t GLOBAL_HIST_MASK = (1 << L[i]) - 1;
        index_t ghist_bits = index_t(ghist) & GLOBAL_HIST_MASK;
        std::size_t bitsfilled = 0;
        // First add path history
        if (L[i] < PATH_HIST_LENGTH) {
          bitvector |= index_t(phist) & GLOBAL_HIST_MASK;
          bitsfilled += L[i];
        } else {
          bitvector |= index_t(phist);
          bitsfilled += PATH_HIST_LENGTH;
        }
        // Then global branch history
        ghist_bits <<= bitsfilled;
        bitvector |= ghist_bits;
        bitsfilled += L[i];
        // Then PC
        bitvector |= (index_t(pc) << bitsfilled);
        bitsfilled += 32;

        // index = (history_t(pc) & PHT_INDEX_MASK); // XOR Identity
        index = 0;
        index_t BITV_PHT_INDEX_MASK(PHT_INDEX_MASK);
        for (std::size_t j = 0; j < bitsfilled; j += PHT_SIZES[i]) {
          index ^= (bitvector & BITV_PHT_INDEX_MASK).to_ulong();
          bitvector >>= PHT_SIZES[i];
        }
      }
      indices[i] = index;
    }
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

      prediction = get_gehl_pred(pc);

    }
    return prediction;   // true for taken, false for not taken
  }

  bool get_gehl_pred(address_t pc) {
    calc_indices(pc);
    calc_sum();
    return sum >= 0;
  }

  double calc_sum() {
    // double sum = NUM_TABLES / 2;
    sum = 0;

    for (int i = 0; i < NUM_TABLES; ++i) {
      sum += ((double)pht[i][indices[i]])/COUNTER_BITS[i];
    }

    return sum;
  }

  void update_counters(bool taken) {
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

  void update_gehl_predictor(bool taken) {
    bool pred = sum >= 0;
    if (pred != taken || abs(sum) < THRESH) {
      update_counters(taken);
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
  }

  // Update the predictor after a prediction has been made.  This should accept
  // the branch record (br) and architectural state (os), as well as a third
  // argument (taken) indicating whether or not the branch was taken.
  void update_predictor(const branch_record_c* br, const op_state_c*, bool taken) {

    address_t pc =  br->instruction_addr;
    if (/* conditional branch */ br->is_conditional) {
      update_gehl_predictor(taken);
      update_ghist(taken);
      update_phist(pc & 1);
    } else if (br->is_call || br->is_return || br->is_indirect) {
      update_ghist(true);
      update_phist(pc & 1);
    }

  }
};

#endif // PREDICTOR_H_SEEN

