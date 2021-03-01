#ifndef RTW_HEADER_Autothrust_h_
#define RTW_HEADER_Autothrust_h_
#include <cmath>
#include <cstring>
#ifndef Autothrust_COMMON_INCLUDES_
# define Autothrust_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "Autothrust_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_a;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_h;
  real_T Delay_DSTATE_n;
  real_T Delay_DSTATE_l;
  real_T Delay_DSTATE_lz;
  real_T Delay_DSTATE_he;
  real_T PrevY;
  real_T prev_TLA_1;
  real_T prev_TLA_2;
  real_T eventTime;
  real_T eventTime_n;
  athr_mode pMode;
  boolean_T Delay_DSTATE_b;
  uint8_T icLoad;
  uint8_T is_active_c5_Autothrust;
  uint8_T is_c5_Autothrust;
  boolean_T Memory_PreviousInput;
  boolean_T Memory_PreviousInput_f;
  boolean_T ATHR_ENGAGED;
  boolean_T prev_TLA_1_not_empty;
  boolean_T prev_TLA_2_not_empty;
  boolean_T pThrustMemoActive;
  boolean_T pUseAutoThrustControl;
  boolean_T eventTime_not_empty;
  boolean_T eventTime_not_empty_i;
} D_Work_Autothrust_T;

typedef struct {
  athr_in in;
} ExternalInputs_Autothrust_T;

typedef struct {
  athr_out out;
} ExternalOutputs_Autothrust_T;

struct Parameters_Autothrust_T_ {
  athr_out athr_out_MATLABStruct;
  real_T LagFilter_C1;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteTimeIntegratorVariableTsLimit_Gain;
  real_T DiscreteTimeIntegratorVariableTs_Gain;
  real_T DiscreteTimeIntegratorVariableTs1_Gain;
  real_T DiscreteTimeIntegratorVariableTs_Gain_k;
  real_T DiscreteTimeIntegratorVariableTs1_Gain_l;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T DiscreteTimeIntegratorVariableTs_InitialCondition;
  real_T DiscreteTimeIntegratorVariableTs1_InitialCondition;
  real_T DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  real_T DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  real_T DiscreteTimeIntegratorVariableTs_LowerLimit;
  real_T DiscreteTimeIntegratorVariableTs1_LowerLimit;
  real_T DiscreteTimeIntegratorVariableTs_LowerLimit_e;
  real_T DiscreteTimeIntegratorVariableTs1_LowerLimit_h;
  real_T DiscreteTimeIntegratorVariableTs_UpperLimit;
  real_T DiscreteTimeIntegratorVariableTs1_UpperLimit;
  real_T DiscreteTimeIntegratorVariableTs_UpperLimit_p;
  real_T DiscreteTimeIntegratorVariableTs1_UpperLimit_o;
  real_T CompareToConstant_const;
  real_T CompareToConstant2_const;
  athr_status CompareToConstant_const_g;
  boolean_T CompareToConstant1_const;
  boolean_T CompareToConstant_const_o;
  boolean_T SRFlipFlop_initial_condition;
  boolean_T SRFlipFlop_initial_condition_a;
  real_T Gain_Gain;
  real_T Gain_Gain_f;
  real_T Gain1_Gain;
  real_T Gain_Gain_o;
  real_T Gain_Gain_b;
  real_T Gain2_Gain;
  real_T Gain1_Gain_h;
  real_T Constant1_Value;
  real_T Gain_Gain_p;
  real_T Constant1_Value_d;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_f;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Gain3_Gain;
  real_T Delay_InitialCondition;
  real_T Constant_Value;
  real_T Delay1_InitialCondition;
  real_T RateLimiter_RisingLim;
  real_T RateLimiter_FallingLim;
  real_T RateLimiter_IC;
  real_T Gain_Gain_d;
  real_T Gain1_Gain_hu;
  real_T Gain_Gain_bf;
  real_T Gain1_Gain_g;
  boolean_T Logic_table[16];
  boolean_T Delay_InitialCondition_a;
  boolean_T Logic_table_g[16];
  boolean_T Constant_Value_l;
};

extern const athr_in Autothrust_rtZathr_in;
extern const athr_out Autothrust_rtZathr_out;
class AutothrustModelClass {
 public:
  ExternalInputs_Autothrust_T Autothrust_U;
  ExternalOutputs_Autothrust_T Autothrust_Y;
  void initialize();
  void step();
  void terminate();
  AutothrustModelClass();
  ~AutothrustModelClass();
 private:
  static Parameters_Autothrust_T Autothrust_P;
  D_Work_Autothrust_T Autothrust_DWork;
  void Autothrust_TLAComputation1(const athr_out *rtu_in, real_T rtu_TLA, real_T *rty_N1c, boolean_T *rty_inReverse);
  void Autothrust_ThrustMode1(real_T rtu_u, real_T *rty_y);
};

#endif

