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


class loop_entry {
public:
  uint16_t PastIter;		// 10 bits
  uint8_t conf;		      // 2 bits
  uint16_t CurIter;		  // 10 bits

  uint16_t TAG;			    // 12 bits
  uint8_t age;			    // 4 bits
  bool dir;			        // 1 bit

  // 39 bits per entry
  loop_entry () {
    conf = 0;
    CurIter = 0;
    PastIter = 0;
    TAG = 0;
    age = 0;
    dir = false;
  }

};

class PREDICTOR {
public:
  typedef uint32_t address_t;

private:
  typedef uint64_t path_t;
  typedef unsigned __int128 history_t;
  typedef int8_t counter_t;

  // Constant Definitions
  static const int NUM_TABLES = 8;
  const size_t L[NUM_TABLES] = {0, 2, 4, 8, 16, 32, 64, 128};
  const size_t PHT_SIZES[NUM_TABLES] = {11, 10, 11, 11, 11, 11, 11, 11};
  const size_t COUNTER_BITS[NUM_TABLES] = {5, 5, 4, 4, 4, 4, 4, 4};

  // Path History
  static const int PATH_HIST_LENGTH = 48;  // 48 bits
  static const path_t PATH_HIST_MASK = ((unsigned __int128)(1) << PATH_HIST_LENGTH) - 1;

  // Global History
  static const int GLOBAL_HIST_LENGTH = (1 << (NUM_TABLES - 1));  // 128 bits
  static const counter_t PHT_INIT = /* very weakly taken */ 0;

  // Loop Predictor
  static const int LOOP_PRED_SIZE  = 5;  // 32 entries
  static const int WIDTH_ITER_LOOP = 10; // we predict only loops with less than 1K iterations
  static const int LOOP_TAG_WIDTH  = 12; // tag width in the loop predictor
  static const int LOOP_CONFIDENCE = 3;  // Max Confidence in a loop prediction
  static const int MAX_AGE         = 15; // Max Age of a loop prediction
  static const int WITHLOOP_WIDTH  = 7;  // Counter width of the WITHLOOP counter

  static counter_t counter_inc(/* n-bit counter */ counter_t cnt, int n) {
    if (cnt != (1 << (n - 1)) - 1)
      ++cnt;
    return cnt;
  }
  static counter_t counter_dec(/* n-bit counter */ counter_t cnt, int n) {
    if (cnt != -(1 << (n - 1)))
      --cnt;
    return cnt;
  }

  // Hardware Data Structures

  // Global History Register
  history_t ghist;                         // 128 bits
  // Path History Register to prevent path aliasing
  path_t phist;                            // 48 bits
  // Various Pattern History Tables indexed by History Length
  std::vector<counter_t> pht[NUM_TABLES];  // 1 x 2K x 5 + 1 x 1K x 5 + 6 x 2K x 4 = 63K
  // Loop Predictor Table
  loop_entry *ltable;			                 // 39 * 32 bits = 1248 bits
  // Counter to monitor whether or not loop prediction is beneficial
  int8_t WITHLOOP;		                     // 7 bits
  // A seed for generating randomness
  int Seed;                                // 32 bits
  // Threshold for updating gehl predictors
  counter_t THRESH;                        // Log(NUM_TABLES) = 3 bit
  // Counter for dynamic thresholding
  counter_t TC;                            // 7 bits

  // Total = 65985 bits < 64K + 512 bits = 66048

  // Per Branch Variables used in both getting and updating prediction
  std::size_t indices[NUM_TABLES];         // Indices to the pht
  double sum;                              // Adder sum
  bool prediction;                         // Prediction of this particular branch

  // Variables for the loop predictor
  bool LVALID;			          // validity of the loop predictor prediction
  bool predloop;			        // loop predictor prediction
  int LIB;
  int LI;
  int LHIT;			      // hitting way in the loop predictor
  int LTAG;			      // tag on the loop predictor

  void update_ghist(bool taken) {
    ghist <<= 1;
    // ghist &= GLOBAL_HIST_MASK; // Not needed as max width of ghist is 128 bits
    if (taken)
      ghist |= history_t(1);
  }

  void update_phist(path_t addr_bit) {
    phist <<= 1;
    phist &= PATH_HIST_MASK;
    phist |= addr_bit;
  }

public:
  PREDICTOR(void)
    : ghist(0)
    , phist(0)
    , ltable(new loop_entry[1 << LOOP_PRED_SIZE])
    , WITHLOOP(-1)
    , Seed(0)
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
    prediction = false;
    if (/* conditional branch */ br->is_conditional) {
      address_t pc = br->instruction_addr;

      prediction = get_gehl_pred(pc);

      predloop = get_loop_pred(pc);	// loop prediction
      prediction = ((WITHLOOP >= 0) && (LVALID)) ? predloop : prediction;

    }
    return prediction;   // true for taken, false for not taken
  }

  int lindex(address_t pc) {
    return (((pc ^ (pc >> 2)) & ((1 << (LOOP_PRED_SIZE - 2)) - 1)) << 2);
  }

  // loop prediction: only used if high confidence
  // skewed associative 4-way
  // At fetch time: speculative
  bool get_loop_pred(address_t pc) {
      LHIT = -1;

      LI = lindex (pc);
      LIB = ((pc >> (LOOP_PRED_SIZE - 2)) & ((1 << (LOOP_PRED_SIZE - 2)) - 1));
      LTAG = (pc >> (LOOP_PRED_SIZE - 2)) & ((1 << 2 * LOOP_TAG_WIDTH) - 1);
      LTAG ^= (LTAG >> LOOP_TAG_WIDTH);
      LTAG = (LTAG & ((1 << LOOP_TAG_WIDTH) - 1));

      for (int i = 0; i < 4; i++) {
        int index = (LI ^ ((LIB >> i) << 2)) + i;
        if (ltable[index].TAG == LTAG) {
          LHIT = i;
          LVALID = ((ltable[index].conf == LOOP_CONFIDENCE)
                    || (ltable[index].conf * ltable[index].PastIter > 128));
          if (ltable[index].CurIter + 1 == ltable[index].PastIter) {
            return !(ltable[index].dir);
          }
          return ltable[index].dir;
        }
      }
      LVALID = false;
      return false;
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
        if (THRESH != NUM_TABLES)
          ++THRESH;
        TC = 0;
      }
    }
    if ((pred == taken) && (abs(sum) < THRESH)) {
      --TC;
      if (TC == -64) {
        if (THRESH != 0)
          --THRESH;
        TC = 0;
      }
    }
  }

  int MYRANDOM () {
    Seed++;
    Seed ^= phist;
    Seed = (Seed >> 21) + (Seed << 11);
    Seed ^= ghist;
    Seed = (Seed >> 10) + (Seed << 22);
    return (Seed);
  };


  void update_loop_predictor (address_t, bool taken, bool alloc) {
    bool gehl_prediction = sum >= 0;
    if (LHIT >= 0) {
      int index = (LI ^ ((LIB >> LHIT) << 2)) + LHIT;
      //already a hit
      if (LVALID) {
        if (taken != predloop) {
          // free the entry
          ltable[index].PastIter = 0;
          ltable[index].age = 0;
          ltable[index].conf = 0;
          ltable[index].CurIter = 0;
          return;
	      }	else if ((predloop != gehl_prediction) || ((MYRANDOM () & 7) == 0))
          if (ltable[index].age < MAX_AGE)
            ltable[index].age++;
      }

      ltable[index].CurIter++;
      ltable[index].CurIter &= ((1 << WIDTH_ITER_LOOP) - 1);
      // loop with more than 2** WIDTH_ITER_LOOP iterations are not treated correctly; but who cares :-)
      if (ltable[index].CurIter > ltable[index].PastIter) {
        ltable[index].conf = 0;
        ltable[index].PastIter = 0;
        // treat like the 1st encounter of the loop
      }
      if (taken != ltable[index].dir) {
        if (ltable[index].CurIter == ltable[index].PastIter) {
          if (ltable[index].conf < LOOP_CONFIDENCE)
            ltable[index].conf++;
          //just do not predict when the loop count is 1 or 2
          if (ltable[index].PastIter < 3) {
            // free the entry
            ltable[index].dir = taken;
            ltable[index].PastIter = 0;
            ltable[index].age = 0;
            ltable[index].conf = 0;
          }
	      }	else {
          if (ltable[index].PastIter == 0) {
            // first complete nest;
            ltable[index].conf = 0;
            ltable[index].PastIter = ltable[index].CurIter;
          }	else {
            //not the same number of iterations as last time: free the entry
            ltable[index].PastIter = 0;
            ltable[index].conf = 0;
          }
	      }
        ltable[index].CurIter = 0;
      }
    } else if (alloc) {
      address_t X = MYRANDOM () & 3;
      if ((MYRANDOM () & 3) == 0)
        for (int i = 0; i < 4; i++) {
          int LHIT = (X + i) & 3;
          int index = (LI ^ ((LIB >> LHIT) << 2)) + LHIT;
          if (ltable[index].age == 0)	{
            ltable[index].dir = !taken;
            // most of mispredictions are on last iterations
            ltable[index].TAG = LTAG;
            ltable[index].PastIter = 0;
            ltable[index].age = 7;
            ltable[index].conf = 0;
            ltable[index].CurIter = 0;
            break;
          }	else
            ltable[index].age--;
          break;
        }
    }
  }

  // Update the predictor after a prediction has been made.  This should accept
  // the branch record (br) and architectural state (os), as well as a third
  // argument (taken) indicating whether or not the branch was taken.
  void update_predictor(const branch_record_c* br, const op_state_c*, bool taken) {

    address_t pc =  br->instruction_addr;
    if (/* conditional branch */ br->is_conditional) {

      if (LVALID) {
        if (prediction != predloop) {
          if (predloop == taken) {
            WITHLOOP = counter_inc(WITHLOOP, WITHLOOP_WIDTH);
          } else {
            WITHLOOP = counter_dec(WITHLOOP, WITHLOOP_WIDTH);
          }
        }
      }
      update_loop_predictor(pc, taken, (prediction != taken));

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

