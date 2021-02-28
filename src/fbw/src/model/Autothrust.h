#ifndef RTW_HEADER_Autothrust_h_
#define RTW_HEADER_Autothrust_h_
#include <cstring>
#ifndef Autothrust_COMMON_INCLUDES_
# define Autothrust_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "Autothrust_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_l;
  real_T Delay_DSTATE_lz;
  real_T Delay_DSTATE_h;
  uint8_T is_active_c5_Autothrust;
  uint8_T is_c5_Autothrust;
} D_Work_Autothrust_T;

typedef struct {
  athr_in in;
} ExternalInputs_Autothrust_T;

typedef struct {
  athr_out out;
} ExternalOutputs_Autothrust_T;

struct Parameters_Autothrust_T_ {
  athr_out athr_out_MATLABStruct;
  real_T DiscreteTimeIntegratorVariableTs_Gain;
  real_T DiscreteTimeIntegratorVariableTs1_Gain;
  real_T DiscreteTimeIntegratorVariableTs_Gain_k;
  real_T DiscreteTimeIntegratorVariableTs1_Gain_l;
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
  real_T CompareToConstant_const_a;
  real_T Gain_Gain;
  real_T Constant1_Value;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T uDLookupTable_tableData[5];
  real_T uDLookupTable_bp01Data[5];
  real_T uDLookupTable_tableData_i[5];
  real_T uDLookupTable_bp01Data_l[5];
  real_T Gain_Gain_d;
  real_T Gain1_Gain_h;
  real_T Gain_Gain_b;
  real_T Gain1_Gain_g;
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
  void Autothrust_ThrustMode1(real_T rtu_u, real_T *rty_y);
};

#endif

