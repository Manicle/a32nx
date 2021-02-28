#include "Autothrust.h"
#include "Autothrust_private.h"
#include "look1_binlxpw.h"

const uint8_T Autothrust_IN_InAir = 1U;
const uint8_T Autothrust_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T Autothrust_IN_OnGround = 2U;
const athr_out Autothrust_rtZathr_out = {
  {
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    false,
    false,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    false,
    athr_thrust_limit_type_NONE,
    0.0,
    0.0,
    0.0,
    athr_status_DISENGAGED,
    athr_mode_NONE,
    athr_mode_message_NONE
  }
} ;

const athr_in Autothrust_rtZathr_in = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, false, false, 0.0, 0.0, 0.0, 0.0 }, { false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 } };

void AutothrustModelClass::Autothrust_ThrustMode1(real_T rtu_u, real_T *rty_y)
{
  if (rtu_u < 0.0) {
    *rty_y = 1.0;
  } else if (rtu_u == 0.0) {
    *rty_y = 2.0;
  } else if ((rtu_u > 0.0) && (rtu_u < 89.0)) {
    *rty_y = 3.0;
  } else if ((rtu_u >= 89.0) && (rtu_u < 95.0)) {
    *rty_y = 4.0;
  } else if ((rtu_u >= 95.0) && (rtu_u < 100.0)) {
    *rty_y = 5.0;
  } else if (rtu_u == 100.0) {
    *rty_y = 6.0;
  } else {
    *rty_y = 0.0;
  }
}

void AutothrustModelClass::step()
{
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  boolean_T rtb_Compare;
  boolean_T rtb_BusAssignment_output_is_in_reverse_2;
  rtb_Saturation = Autothrust_P.Gain_Gain * Autothrust_U.in.data.gear_strut_compression_1 - Autothrust_P.Constant1_Value;
  if (rtb_Saturation > Autothrust_P.Saturation_UpperSat) {
    rtb_Saturation = Autothrust_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < Autothrust_P.Saturation_LowerSat) {
      rtb_Saturation = Autothrust_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = Autothrust_P.Gain1_Gain * Autothrust_U.in.data.gear_strut_compression_2 -
    Autothrust_P.Constant1_Value;
  if (rtb_Saturation1 > Autothrust_P.Saturation1_UpperSat) {
    rtb_Saturation1 = Autothrust_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < Autothrust_P.Saturation1_LowerSat) {
      rtb_Saturation1 = Autothrust_P.Saturation1_LowerSat;
    }
  }

  if (Autothrust_DWork.is_active_c5_Autothrust == 0U) {
    Autothrust_DWork.is_active_c5_Autothrust = 1U;
    Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_OnGround;
    rtb_on_ground = 1;
  } else if (Autothrust_DWork.is_c5_Autothrust == Autothrust_IN_InAir) {
    if ((rtb_Saturation > 0.05) || (rtb_Saturation1 > 0.05)) {
      Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_Saturation1 = look1_binlxpw(Autothrust_U.in.input.TLA_1_deg, Autothrust_P.uDLookupTable_bp01Data,
    Autothrust_P.uDLookupTable_tableData, 4U);
  rtb_Compare = (Autothrust_U.in.input.TLA_1_deg < Autothrust_P.CompareToConstant_const);
  Autothrust_Y.out.output.N1_TLA_1_percent = rtb_Saturation1;
  rtb_Saturation = look1_binlxpw(Autothrust_U.in.input.TLA_2_deg, Autothrust_P.uDLookupTable_bp01Data_l,
    Autothrust_P.uDLookupTable_tableData_i, 4U);
  rtb_BusAssignment_output_is_in_reverse_2 = (Autothrust_U.in.input.TLA_2_deg < Autothrust_P.CompareToConstant_const_a);
  rtb_Saturation1 -= Autothrust_U.in.data.commanded_engine_N1_1_percent;
  if (rtb_Compare) {
    Autothrust_DWork.Delay_DSTATE = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE += Autothrust_P.Gain_Gain_d * rtb_Saturation1 *
    Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  if (!rtb_Compare) {
    Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE_l += Autothrust_P.Gain1_Gain_h * rtb_Saturation1 *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_l > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_l < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_1_deg, &Autothrust_Y.out.output.sim_thrust_mode_1);
  rtb_Saturation1 = rtb_Saturation - Autothrust_U.in.data.commanded_engine_N1_2_percent;
  if (rtb_BusAssignment_output_is_in_reverse_2) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  }

  Autothrust_DWork.Delay_DSTATE_lz += Autothrust_P.Gain_Gain_b * rtb_Saturation1 *
    Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain_k * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_lz > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_lz < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e) {
      Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e;
    }
  }

  if (!rtb_BusAssignment_output_is_in_reverse_2) {
    Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  }

  Autothrust_DWork.Delay_DSTATE_h += Autothrust_P.Gain1_Gain_g * rtb_Saturation1 *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain_l * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_h > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o) {
    Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_h < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h) {
      Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_2_deg, &rtb_Saturation1);
  Autothrust_Y.out.time = Autothrust_U.in.time;
  Autothrust_Y.out.data.nz_g = Autothrust_U.in.data.nz_g;
  Autothrust_Y.out.data.V_ias_kn = Autothrust_U.in.data.V_ias_kn;
  Autothrust_Y.out.data.V_tas_kn = Autothrust_U.in.data.V_tas_kn;
  Autothrust_Y.out.data.V_mach = Autothrust_U.in.data.V_mach;
  Autothrust_Y.out.data.V_gnd_kn = Autothrust_U.in.data.V_gnd_kn;
  Autothrust_Y.out.data.alpha_deg = Autothrust_U.in.data.alpha_deg;
  Autothrust_Y.out.data.H_ft = Autothrust_U.in.data.H_ft;
  Autothrust_Y.out.data.H_ind_ft = Autothrust_U.in.data.H_ind_ft;
  Autothrust_Y.out.data.H_radio_ft = Autothrust_U.in.data.H_radio_ft;
  Autothrust_Y.out.data.H_dot_fpm = Autothrust_U.in.data.H_dot_fpm;
  Autothrust_Y.out.data.bx_m_s2 = Autothrust_U.in.data.bx_m_s2;
  Autothrust_Y.out.data.by_m_s2 = Autothrust_U.in.data.by_m_s2;
  Autothrust_Y.out.data.bz_m_s2 = Autothrust_U.in.data.bz_m_s2;
  Autothrust_Y.out.data.on_ground = (rtb_on_ground != 0);
  Autothrust_Y.out.data.flap_handle_index = Autothrust_U.in.data.flap_handle_index;
  Autothrust_Y.out.data.is_engine_operative_1 = Autothrust_U.in.data.is_engine_operative_1;
  Autothrust_Y.out.data.is_engine_operative_2 = Autothrust_U.in.data.is_engine_operative_2;
  Autothrust_Y.out.data.commanded_engine_N1_1_percent = Autothrust_U.in.data.commanded_engine_N1_1_percent;
  Autothrust_Y.out.data.commanded_engine_N1_2_percent = Autothrust_U.in.data.commanded_engine_N1_2_percent;
  Autothrust_Y.out.data.engine_N1_1_percent = Autothrust_U.in.data.engine_N1_1_percent;
  Autothrust_Y.out.data.engine_N1_2_percent = Autothrust_U.in.data.engine_N1_2_percent;
  Autothrust_Y.out.input = Autothrust_U.in.input;
  if (!rtb_Compare) {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE_l;
  }

  if (!rtb_BusAssignment_output_is_in_reverse_2) {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_lz;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_h;
  }

  Autothrust_Y.out.output.sim_thrust_mode_2 = rtb_Saturation1;
  Autothrust_Y.out.output.N1_TLA_2_percent = rtb_Saturation;
  Autothrust_Y.out.output.is_in_reverse_1 = rtb_Compare;
  Autothrust_Y.out.output.is_in_reverse_2 = rtb_BusAssignment_output_is_in_reverse_2;
  Autothrust_Y.out.output.thrust_limit_type = Autothrust_P.athr_out_MATLABStruct.output.thrust_limit_type;
  Autothrust_Y.out.output.thrust_limit_percent = Autothrust_P.athr_out_MATLABStruct.output.thrust_limit_percent;
  Autothrust_Y.out.output.N1_c_1_percent = Autothrust_P.athr_out_MATLABStruct.output.N1_c_1_percent;
  Autothrust_Y.out.output.N1_c_2_percent = Autothrust_P.athr_out_MATLABStruct.output.N1_c_2_percent;
  Autothrust_Y.out.output.status = Autothrust_P.athr_out_MATLABStruct.output.status;
  Autothrust_Y.out.output.mode = Autothrust_P.athr_out_MATLABStruct.output.mode;
  Autothrust_Y.out.output.mode_message = Autothrust_P.athr_out_MATLABStruct.output.mode_message;
}

void AutothrustModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&Autothrust_DWork), 0,
                     sizeof(D_Work_Autothrust_T));
  Autothrust_U.in = Autothrust_rtZathr_in;
  Autothrust_Y.out = Autothrust_rtZathr_out;
  Autothrust_DWork.Delay_DSTATE = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  Autothrust_DWork.is_active_c5_Autothrust = 0U;
  Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_NO_ACTIVE_CHILD;
}

void AutothrustModelClass::terminate()
{
}

AutothrustModelClass::AutothrustModelClass()
{
}

AutothrustModelClass::~AutothrustModelClass()
{
}
