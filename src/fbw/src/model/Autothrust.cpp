#include "Autothrust.h"
#include "Autothrust_private.h"

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
    false,
    false,
    false
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
    0.0,
    false,
    false,
    false,
    false,
    false,
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
    0.0, false, false, false, false, false, 0.0, 0.0 } };

void AutothrustModelClass::Autothrust_TLAComputation1(const athr_out *rtu_in, real_T rtu_TLA, real_T *rty_N1c, boolean_T
  *rty_inReverse)
{
  int32_T TLA_begin;
  real_T N1_begin;
  int32_T TLA_end;
  real_T N1_end;
  real_T u0;
  u0 = rtu_TLA;
  *rty_inReverse = (rtu_TLA < 0.0);
  if (rtu_TLA >= 0.0) {
    if (rtu_TLA <= 25.0) {
      TLA_begin = 0;
      N1_begin = rtu_in->input.thrust_limit_IDLE_percent;
      TLA_end = 25;
      N1_end = rtu_in->input.thrust_limit_CLB_percent;
    } else if (rtu_TLA <= 35.0) {
      TLA_begin = 25;
      N1_begin = rtu_in->input.thrust_limit_CLB_percent;
      TLA_end = 35;
      if (rtu_in->data_computed.is_FLX_active) {
        N1_end = rtu_in->input.thrust_limit_FLEX_percent;
      } else {
        N1_end = rtu_in->input.thrust_limit_MCT_percent;
      }
    } else {
      TLA_begin = 35;
      if (rtu_in->data_computed.is_FLX_active) {
        N1_begin = rtu_in->input.thrust_limit_FLEX_percent;
      } else {
        N1_begin = rtu_in->input.thrust_limit_MCT_percent;
      }

      TLA_end = 45;
      N1_end = rtu_in->input.thrust_limit_TOGA_percent;
    }
  } else {
    u0 = std::abs(rtu_TLA);
    if (u0 <= 6.0) {
      u0 = 6.0;
    }

    TLA_begin = 6;
    N1_begin = std::abs(rtu_in->input.thrust_limit_IDLE_percent);
    TLA_end = 20;
    N1_end = std::abs(rtu_in->input.thrust_limit_REV_percent);
  }

  *rty_N1c = (N1_end - N1_begin) / static_cast<real_T>((TLA_end - TLA_begin)) * (u0 - static_cast<real_T>(TLA_begin)) +
    N1_begin;
}

void AutothrustModelClass::Autothrust_ThrustMode1(real_T rtu_u, real_T *rty_y)
{
  if (rtu_u < 0.0) {
    *rty_y = 1.0;
  } else if (rtu_u == 0.0) {
    *rty_y = 2.0;
  } else if ((rtu_u > 0.0) && (rtu_u < 25.0)) {
    *rty_y = 3.0;
  } else if ((rtu_u >= 25.0) && (rtu_u < 35.0)) {
    *rty_y = 4.0;
  } else if ((rtu_u >= 35.0) && (rtu_u < 45.0)) {
    *rty_y = 5.0;
  } else if (rtu_u == 45.0) {
    *rty_y = 6.0;
  } else {
    *rty_y = 0.0;
  }
}

void AutothrustModelClass::step()
{
  int32_T rowIdx;
  int32_T rowIdx_0;
  real_T rtb_Switch_d;
  real_T rtb_Switch_fs;
  real_T rtb_y_b;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  athr_out rtb_BusAssignment;
  boolean_T rtb_inReverse;
  boolean_T rtb_NOT1;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push;
  boolean_T rtb_out;
  athr_status rtb_status;
  real_T rtb_Gain;
  real_T rtb_Sum3;
  real_T rtb_Switch_f_idx_0;
  real_T rtb_Switch_f_idx_1;
  boolean_T ATHR_ENGAGED_tmp;
  boolean_T ATHR_ENGAGED_tmp_0;
  boolean_T ATHR_ENGAGED_tmp_1;
  rtb_Saturation = Autothrust_P.Gain_Gain_p * Autothrust_U.in.data.gear_strut_compression_1 -
    Autothrust_P.Constant1_Value_d;
  if (rtb_Saturation > Autothrust_P.Saturation_UpperSat) {
    rtb_Saturation = Autothrust_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < Autothrust_P.Saturation_LowerSat) {
      rtb_Saturation = Autothrust_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = Autothrust_P.Gain1_Gain_f * Autothrust_U.in.data.gear_strut_compression_2 -
    Autothrust_P.Constant1_Value_d;
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

  rtb_BusAssignment = Autothrust_P.athr_out_MATLABStruct;
  rtb_BusAssignment.time = Autothrust_U.in.time;
  rtb_BusAssignment.data.nz_g = Autothrust_U.in.data.nz_g;
  rtb_BusAssignment.data.V_ias_kn = Autothrust_U.in.data.V_ias_kn;
  rtb_BusAssignment.data.V_tas_kn = Autothrust_U.in.data.V_tas_kn;
  rtb_BusAssignment.data.V_mach = Autothrust_U.in.data.V_mach;
  rtb_BusAssignment.data.V_gnd_kn = Autothrust_U.in.data.V_gnd_kn;
  rtb_BusAssignment.data.alpha_deg = Autothrust_U.in.data.alpha_deg;
  rtb_BusAssignment.data.H_ft = Autothrust_U.in.data.H_ft;
  rtb_BusAssignment.data.H_ind_ft = Autothrust_U.in.data.H_ind_ft;
  rtb_BusAssignment.data.H_radio_ft = Autothrust_U.in.data.H_radio_ft;
  rtb_BusAssignment.data.H_dot_fpm = Autothrust_U.in.data.H_dot_fpm;
  rtb_BusAssignment.data.bx_m_s2 = Autothrust_U.in.data.bx_m_s2;
  rtb_BusAssignment.data.by_m_s2 = Autothrust_U.in.data.by_m_s2;
  rtb_BusAssignment.data.bz_m_s2 = Autothrust_U.in.data.bz_m_s2;
  rtb_BusAssignment.data.on_ground = (rtb_on_ground != 0);
  rtb_BusAssignment.data.flap_handle_index = Autothrust_U.in.data.flap_handle_index;
  rtb_BusAssignment.data.is_engine_operative_1 = Autothrust_U.in.data.is_engine_operative_1;
  rtb_BusAssignment.data.is_engine_operative_2 = Autothrust_U.in.data.is_engine_operative_2;
  rtb_BusAssignment.data.commanded_engine_N1_1_percent = Autothrust_U.in.data.commanded_engine_N1_1_percent;
  rtb_BusAssignment.data.commanded_engine_N1_2_percent = Autothrust_U.in.data.commanded_engine_N1_2_percent;
  rtb_BusAssignment.data.engine_N1_1_percent = Autothrust_U.in.data.engine_N1_1_percent;
  rtb_BusAssignment.data.engine_N1_2_percent = Autothrust_U.in.data.engine_N1_2_percent;
  rtb_BusAssignment.input = Autothrust_U.in.input;
  if (!Autothrust_DWork.eventTime_not_empty) {
    Autothrust_DWork.eventTime = Autothrust_U.in.time.simulation_time;
    Autothrust_DWork.eventTime_not_empty = true;
  }

  if ((!Autothrust_U.in.input.ATHR_push) || (Autothrust_DWork.eventTime == 0.0)) {
    Autothrust_DWork.eventTime = Autothrust_U.in.time.simulation_time;
  }

  rowIdx = static_cast<int32_T>(((((static_cast<uint32_T>((Autothrust_U.in.time.simulation_time -
    Autothrust_DWork.eventTime >= Autothrust_P.CompareToConstant_const)) << 1) + false) << 1) +
    Autothrust_DWork.Memory_PreviousInput));
  rtb_inReverse = Autothrust_DWork.Delay_DSTATE_b;
  if (!Autothrust_DWork.eventTime_not_empty_i) {
    Autothrust_DWork.eventTime_n = Autothrust_U.in.time.simulation_time;
    Autothrust_DWork.eventTime_not_empty_i = true;
  }

  if ((Autothrust_U.in.input.ATHR_push != Autothrust_P.CompareToConstant1_const) || (Autothrust_DWork.eventTime_n == 0.0))
  {
    Autothrust_DWork.eventTime_n = Autothrust_U.in.time.simulation_time;
  }

  rowIdx_0 = static_cast<int32_T>(((((Autothrust_U.in.time.simulation_time - Autothrust_DWork.eventTime_n >=
    Autothrust_P.CompareToConstant2_const) + (static_cast<uint32_T>(Autothrust_DWork.Delay_DSTATE_b) << 1)) << 1) +
    Autothrust_DWork.Memory_PreviousInput_f));
  if (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2) {
    rtb_out = ((Autothrust_U.in.input.TLA_1_deg >= 0.0) && (Autothrust_U.in.input.TLA_1_deg <= 25.0) &&
               (Autothrust_U.in.input.TLA_2_deg >= 0.0) && (Autothrust_U.in.input.TLA_2_deg <= 25.0));
  } else {
    rtb_out = ((Autothrust_U.in.data.is_engine_operative_1 && (Autothrust_U.in.input.TLA_1_deg >= 0.0) &&
                (Autothrust_U.in.input.TLA_1_deg <= 35.0)) || (Autothrust_U.in.data.is_engine_operative_2 &&
                (Autothrust_U.in.input.TLA_2_deg >= 0.0) && (Autothrust_U.in.input.TLA_2_deg <= 35.0)));
  }

  Autothrust_DWork.Delay_DSTATE_b = (static_cast<int32_T>(Autothrust_U.in.input.ATHR_push) > static_cast<int32_T>
    (Autothrust_P.CompareToConstant_const_o));
  rtb_NOT1 = (Autothrust_DWork.Delay_DSTATE_b && (!Autothrust_P.Logic_table_g[static_cast<uint32_T>(rowIdx_0)]));
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push = rtb_NOT1;
  Autothrust_TLAComputation1(&rtb_BusAssignment, Autothrust_U.in.input.TLA_1_deg, &rtb_Switch_fs, &rtb_inReverse);
  Autothrust_TLAComputation1(&rtb_BusAssignment, Autothrust_U.in.input.TLA_2_deg, &rtb_Saturation, &rtb_NOT1);
  if (!Autothrust_DWork.prev_TLA_1_not_empty) {
    Autothrust_DWork.prev_TLA_1 = Autothrust_U.in.input.TLA_1_deg;
    Autothrust_DWork.prev_TLA_1_not_empty = true;
  }

  if (!Autothrust_DWork.prev_TLA_2_not_empty) {
    Autothrust_DWork.prev_TLA_2 = Autothrust_U.in.input.TLA_2_deg;
    Autothrust_DWork.prev_TLA_2_not_empty = true;
  }

  ATHR_ENGAGED_tmp = !Autothrust_DWork.ATHR_ENGAGED;
  ATHR_ENGAGED_tmp_0 = ((Autothrust_DWork.prev_TLA_1 <= 0.0) || (Autothrust_U.in.input.TLA_1_deg != 0.0));
  ATHR_ENGAGED_tmp_1 = ((Autothrust_DWork.prev_TLA_2 <= 0.0) || (Autothrust_U.in.input.TLA_2_deg != 0.0));
  Autothrust_DWork.ATHR_ENGAGED = (((rtb_on_ground == 0) && ATHR_ENGAGED_tmp &&
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push) ||
    (((Autothrust_U.in.input.TLA_1_deg == 45.0) && (Autothrust_U.in.input.TLA_2_deg == 45.0)) ||
     (Autothrust_P.Constant_Value_l && (Autothrust_U.in.input.TLA_1_deg >= 35.0) && (Autothrust_U.in.input.TLA_2_deg >=
    35.0))) || (((!rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push) || ATHR_ENGAGED_tmp)
                && ((ATHR_ENGAGED_tmp_0 || ATHR_ENGAGED_tmp_1) && (ATHR_ENGAGED_tmp_0 ||
    (Autothrust_U.in.input.TLA_2_deg != 0.0)) && (ATHR_ENGAGED_tmp_1 || (Autothrust_U.in.input.TLA_1_deg != 0.0))) &&
                Autothrust_DWork.ATHR_ENGAGED));
  ATHR_ENGAGED_tmp = (Autothrust_DWork.ATHR_ENGAGED && rtb_out);
  if (Autothrust_DWork.ATHR_ENGAGED && ATHR_ENGAGED_tmp) {
    rtb_status = athr_status_ENGAGED_ACTIVE;
  } else if (Autothrust_DWork.ATHR_ENGAGED && (!ATHR_ENGAGED_tmp)) {
    rtb_status = athr_status_ENGAGED_ARMED;
  } else {
    rtb_status = athr_status_DISENGAGED;
  }

  Autothrust_DWork.prev_TLA_1 = Autothrust_U.in.input.TLA_1_deg;
  Autothrust_DWork.prev_TLA_2 = Autothrust_U.in.input.TLA_2_deg;
  if (Autothrust_U.in.input.TLA_1_deg > Autothrust_U.in.input.TLA_2_deg) {
    rtb_Saturation1 = Autothrust_U.in.input.TLA_1_deg;
  } else {
    rtb_Saturation1 = Autothrust_U.in.input.TLA_2_deg;
  }

  if (rtb_status == athr_status_DISENGAGED) {
    Autothrust_DWork.pMode = athr_mode_NONE;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((Autothrust_U.in.input.TLA_1_deg == 45.0) ||
              (Autothrust_U.in.input.TLA_2_deg == 45.0))) {
    Autothrust_DWork.pMode = athr_mode_MAN_TOGA;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && Autothrust_P.Constant_Value_l && (rtb_Saturation1 == 35.0)) {
    Autothrust_DWork.pMode = athr_mode_MAN_FLEX;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((Autothrust_U.in.input.TLA_1_deg == 35.0) ||
              (Autothrust_U.in.input.TLA_2_deg == 35.0))) {
    Autothrust_DWork.pMode = athr_mode_MAN_MCT;
  } else {
    ATHR_ENGAGED_tmp = (Autothrust_U.in.data.is_engine_operative_1 && (!Autothrust_U.in.data.is_engine_operative_2));
    ATHR_ENGAGED_tmp_0 = (Autothrust_U.in.data.is_engine_operative_2 && (!Autothrust_U.in.data.is_engine_operative_1));
    if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
        ((ATHR_ENGAGED_tmp && (Autothrust_U.in.input.TLA_1_deg == 35.0) && (Autothrust_U.in.input.TLA_2_deg <= 35.0)) ||
         (ATHR_ENGAGED_tmp_0 && (Autothrust_U.in.input.TLA_2_deg == 35.0) && (Autothrust_U.in.input.TLA_1_deg <= 35.0))))
    {
      Autothrust_DWork.pMode = athr_mode_THR_MCT;
    } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
               Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2 &&
               (rtb_Saturation1 == 25.0)) {
      Autothrust_DWork.pMode = athr_mode_THR_CLB;
    } else {
      ATHR_ENGAGED_tmp_1 = (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2);
      if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
          ((ATHR_ENGAGED_tmp_1 && (Autothrust_U.in.input.TLA_1_deg < 25.0) && (Autothrust_U.in.input.TLA_2_deg < 25.0)) ||
           (ATHR_ENGAGED_tmp && (Autothrust_U.in.input.TLA_1_deg < 35.0)) || (ATHR_ENGAGED_tmp_0 &&
            (Autothrust_U.in.input.TLA_2_deg < 35.0)))) {
        Autothrust_DWork.pMode = athr_mode_THR_LVR;
      } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((ATHR_ENGAGED_tmp_1 && (rtb_Saturation1 > 25.0) &&
                   (rtb_Saturation1 < 35.0)) || ((rtb_Saturation1 > 35.0) && (rtb_Saturation1 < 45.0)))) {
        Autothrust_DWork.pMode = athr_mode_MAN_THR;
      } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 2.0)) {
        Autothrust_DWork.pMode = athr_mode_THR_IDLE;
      } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 1.0) &&
                 (!Autothrust_U.in.input.is_mach_mode_active)) {
        Autothrust_DWork.pMode = athr_mode_SPEED;
      } else {
        if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 1.0) &&
            Autothrust_U.in.input.is_mach_mode_active) {
          Autothrust_DWork.pMode = athr_mode_MACH;
        }
      }
    }
  }

  if ((Autothrust_U.in.input.TLA_1_deg >= 35.0) || (Autothrust_U.in.input.TLA_2_deg >= 35.0)) {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_TOGA;
    Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_TOGA_percent;
  } else if ((Autothrust_U.in.input.TLA_1_deg >= 25.0) || (Autothrust_U.in.input.TLA_2_deg >= 25.0)) {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_MCT;
    Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_MCT_percent;
  } else if ((Autothrust_U.in.input.TLA_1_deg >= 0.0) && (Autothrust_U.in.input.TLA_2_deg >= 0.0)) {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_CLB;
    Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_CLB_percent;
  } else {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_REVERSE;
    Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_REV_percent;
  }

  rtb_Saturation1 = rtb_Switch_fs;
  ATHR_ENGAGED_tmp = ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.TLA_1_deg <= 35.0) &&
                      (Autothrust_U.in.input.TLA_2_deg <= 35.0));
  Autothrust_DWork.pThrustMemoActive = ((((Autothrust_U.in.input.ATHR_push && (rtb_status != athr_status_DISENGAGED)) ||
    ((!ATHR_ENGAGED_tmp) && Autothrust_DWork.pUseAutoThrustControl)) && ((Autothrust_U.in.input.TLA_1_deg == 25.0) ||
    (Autothrust_U.in.input.TLA_1_deg == 35.0)) && ((Autothrust_U.in.input.TLA_2_deg == 25.0) ||
    (Autothrust_U.in.input.TLA_2_deg == 35.0))) || (((Autothrust_U.in.input.TLA_1_deg == 25.0) ||
    (Autothrust_U.in.input.TLA_1_deg == 35.0) || (Autothrust_U.in.input.TLA_2_deg == 25.0) ||
    (Autothrust_U.in.input.TLA_2_deg == 35.0)) && Autothrust_DWork.pThrustMemoActive));
  Autothrust_DWork.pUseAutoThrustControl = ATHR_ENGAGED_tmp;
  if (rtb_Switch_fs > rtb_Saturation) {
    rtb_y_b = rtb_Switch_fs;
  } else {
    rtb_y_b = rtb_Saturation;
  }

  rtb_Gain = Autothrust_P.DiscreteDerivativeVariableTs_Gain * Autothrust_U.in.data.V_ias_kn;
  rtb_Sum3 = (rtb_Gain - Autothrust_DWork.Delay_DSTATE) / Autothrust_U.in.time.dt * Autothrust_P.Gain3_Gain +
    Autothrust_U.in.data.V_ias_kn;
  rtb_Switch_d = Autothrust_U.in.time.dt * Autothrust_P.LagFilter_C1;
  rtb_Switch_fs = rtb_Switch_d + Autothrust_P.Constant_Value;
  Autothrust_DWork.Delay1_DSTATE = 1.0 / rtb_Switch_fs * (Autothrust_P.Constant_Value - rtb_Switch_d) *
    Autothrust_DWork.Delay1_DSTATE + (rtb_Sum3 + Autothrust_DWork.Delay_DSTATE_a) * (rtb_Switch_d / rtb_Switch_fs);
  switch (static_cast<int32_T>(Autothrust_U.in.input.mode_requested)) {
   case 0:
    rtb_Switch_fs = Autothrust_P.Constant1_Value;
    break;

   case 1:
    rtb_Switch_fs = Autothrust_P.Gain1_Gain * Autothrust_U.in.data.alpha_deg;
    rtb_Switch_fs = ((Autothrust_U.in.data.bz_m_s2 * std::sin(rtb_Switch_fs) + std::cos(rtb_Switch_fs) *
                      Autothrust_U.in.data.bx_m_s2) * Autothrust_P.Gain_Gain_o * Autothrust_P.Gain_Gain_b *
                     Autothrust_P.Gain2_Gain + (Autothrust_U.in.input.V_c_kn - Autothrust_DWork.Delay1_DSTATE)) *
      Autothrust_P.Gain1_Gain_h;
    break;

   case 2:
    if (Autothrust_U.in.data.commanded_engine_N1_1_percent > Autothrust_U.in.data.commanded_engine_N1_2_percent) {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_1_percent;
    } else {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_2_percent;
    }

    rtb_Switch_fs = (Autothrust_U.in.input.thrust_limit_IDLE_percent - rtb_Switch_d) * Autothrust_P.Gain_Gain_f;
    break;

   default:
    if (Autothrust_U.in.data.commanded_engine_N1_1_percent > Autothrust_U.in.data.commanded_engine_N1_2_percent) {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_1_percent;
    } else {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_2_percent;
    }

    rtb_Switch_fs = (Autothrust_U.in.input.thrust_limit_CLB_percent - rtb_Switch_d) * Autothrust_P.Gain_Gain;
    break;
  }

  rtb_Switch_fs *= Autothrust_P.DiscreteTimeIntegratorVariableTsLimit_Gain;
  rtb_Switch_fs *= Autothrust_U.in.time.dt;
  if (!(rtb_status == Autothrust_P.CompareToConstant_const_g)) {
    Autothrust_DWork.icLoad = 1U;
  }

  if (Autothrust_DWork.icLoad != 0) {
    if (Autothrust_U.in.data.commanded_engine_N1_1_percent > Autothrust_U.in.data.commanded_engine_N1_2_percent) {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_1_percent;
    } else {
      rtb_Switch_d = Autothrust_U.in.data.commanded_engine_N1_2_percent;
    }

    Autothrust_DWork.Delay_DSTATE_h = rtb_Switch_d - rtb_Switch_fs;
  }

  Autothrust_DWork.Delay_DSTATE_h += rtb_Switch_fs;
  if (Autothrust_DWork.Delay_DSTATE_h > rtb_y_b) {
    Autothrust_DWork.Delay_DSTATE_h = rtb_y_b;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_h < Autothrust_U.in.input.thrust_limit_IDLE_percent) {
      Autothrust_DWork.Delay_DSTATE_h = Autothrust_U.in.input.thrust_limit_IDLE_percent;
    }
  }

  rtb_Switch_fs = Autothrust_DWork.Delay_DSTATE_h - Autothrust_DWork.PrevY;
  if (rtb_Switch_fs > Autothrust_P.RateLimiter_RisingLim) {
    rtb_Switch_fs = Autothrust_DWork.PrevY + Autothrust_P.RateLimiter_RisingLim;
  } else if (rtb_Switch_fs < Autothrust_P.RateLimiter_FallingLim) {
    rtb_Switch_fs = Autothrust_DWork.PrevY + Autothrust_P.RateLimiter_FallingLim;
  } else {
    rtb_Switch_fs = Autothrust_DWork.Delay_DSTATE_h;
  }

  Autothrust_DWork.PrevY = rtb_Switch_fs;
  if (Autothrust_DWork.pUseAutoThrustControl) {
    rtb_Switch_f_idx_0 = rtb_Switch_fs;
    rtb_Switch_f_idx_1 = rtb_Switch_fs;
  } else if (Autothrust_DWork.pThrustMemoActive) {
    rtb_Switch_f_idx_0 = Autothrust_U.in.data.commanded_engine_N1_1_percent;
    rtb_Switch_f_idx_1 = Autothrust_U.in.data.commanded_engine_N1_2_percent;
  } else {
    rtb_Switch_f_idx_0 = rtb_Saturation1;
    rtb_Switch_f_idx_1 = rtb_Saturation;
  }

  Autothrust_Y.out.output.N1_TLA_2_percent = rtb_Saturation;
  rtb_Switch_fs = rtb_Switch_f_idx_0 - Autothrust_U.in.data.commanded_engine_N1_1_percent;
  if (rtb_inReverse) {
    Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE_n += Autothrust_P.Gain_Gain_d * rtb_Switch_fs *
    Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_n > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_n < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  if (!rtb_inReverse) {
    Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE_l += Autothrust_P.Gain1_Gain_hu * rtb_Switch_fs *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_l > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_l < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_1_deg, &rtb_Saturation);
  rtb_Switch_d = rtb_Switch_f_idx_1 - Autothrust_U.in.data.commanded_engine_N1_2_percent;
  if (rtb_NOT1) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  }

  rtb_y_b = Autothrust_P.Gain_Gain_bf * rtb_Switch_d * Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain_k *
    Autothrust_U.in.time.dt + Autothrust_DWork.Delay_DSTATE_lz;
  if (rtb_y_b > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p;
  } else if (rtb_y_b < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e;
  } else {
    Autothrust_DWork.Delay_DSTATE_lz = rtb_y_b;
  }

  if (!rtb_NOT1) {
    Autothrust_DWork.Delay_DSTATE_he = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  }

  Autothrust_DWork.Delay_DSTATE_he += Autothrust_P.Gain1_Gain_g * rtb_Switch_d *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain_l * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_he > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o) {
    Autothrust_DWork.Delay_DSTATE_he = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_he < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h) {
      Autothrust_DWork.Delay_DSTATE_he = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_2_deg, &rtb_y_b);
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
  Autothrust_Y.out.data_computed.TLA_in_active_range = rtb_out;
  Autothrust_Y.out.data_computed.is_FLX_active = Autothrust_P.Constant_Value_l;
  Autothrust_Y.out.data_computed.ATHR_push =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push;
  Autothrust_Y.out.data_computed.ATHR_disabled = Autothrust_P.Logic_table[static_cast<uint32_T>(rowIdx)];
  Autothrust_Y.out.input = Autothrust_U.in.input;
  if (!rtb_inReverse) {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE_n;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE_l;
  }

  if (!rtb_NOT1) {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_lz;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_he;
  }

  Autothrust_Y.out.output.sim_thrust_mode_1 = rtb_Saturation;
  Autothrust_Y.out.output.sim_thrust_mode_2 = rtb_y_b;
  Autothrust_Y.out.output.N1_TLA_1_percent = rtb_Saturation1;
  Autothrust_Y.out.output.is_in_reverse_1 = rtb_inReverse;
  Autothrust_Y.out.output.is_in_reverse_2 = rtb_NOT1;
  Autothrust_Y.out.output.N1_c_1_percent = rtb_Switch_f_idx_0;
  Autothrust_Y.out.output.N1_c_2_percent = rtb_Switch_f_idx_1;
  Autothrust_Y.out.output.status = rtb_status;
  Autothrust_Y.out.output.mode = Autothrust_DWork.pMode;
  Autothrust_Y.out.output.mode_message = athr_mode_message_NONE;
  Autothrust_DWork.Memory_PreviousInput = Autothrust_P.Logic_table[static_cast<uint32_T>(rowIdx)];
  Autothrust_DWork.Memory_PreviousInput_f = Autothrust_P.Logic_table_g[static_cast<uint32_T>(rowIdx_0)];
  Autothrust_DWork.Delay_DSTATE = rtb_Gain;
  Autothrust_DWork.Delay_DSTATE_a = rtb_Sum3;
  Autothrust_DWork.icLoad = 0U;
}

void AutothrustModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&Autothrust_DWork), 0,
                     sizeof(D_Work_Autothrust_T));
  Autothrust_U.in = Autothrust_rtZathr_in;
  Autothrust_Y.out = Autothrust_rtZathr_out;
  Autothrust_DWork.Memory_PreviousInput = Autothrust_P.SRFlipFlop_initial_condition;
  Autothrust_DWork.Delay_DSTATE_b = Autothrust_P.Delay_InitialCondition_a;
  Autothrust_DWork.Memory_PreviousInput_f = Autothrust_P.SRFlipFlop_initial_condition_a;
  Autothrust_DWork.Delay_DSTATE = Autothrust_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_a = Autothrust_P.Delay_InitialCondition;
  Autothrust_DWork.Delay1_DSTATE = Autothrust_P.Delay1_InitialCondition;
  Autothrust_DWork.icLoad = 1U;
  Autothrust_DWork.PrevY = Autothrust_P.RateLimiter_IC;
  Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  Autothrust_DWork.Delay_DSTATE_he = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  Autothrust_DWork.is_active_c5_Autothrust = 0U;
  Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_NO_ACTIVE_CHILD;
  Autothrust_DWork.eventTime_not_empty = false;
  Autothrust_DWork.eventTime_not_empty_i = false;
  Autothrust_DWork.prev_TLA_1_not_empty = false;
  Autothrust_DWork.prev_TLA_2_not_empty = false;
  Autothrust_DWork.ATHR_ENGAGED = false;
  Autothrust_DWork.pMode = athr_mode_NONE;
  Autothrust_DWork.pThrustMemoActive = false;
  Autothrust_DWork.pUseAutoThrustControl = false;
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
