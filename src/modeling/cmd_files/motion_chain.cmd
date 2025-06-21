!
ude create instance  &
   instance_name = .CHAIN_SIMPLE.MOTION_1  &
   definition_name = .MDI.Constraints.general_motion  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.i_marker  &
   object_value = .CHAIN_SIMPLE.body_link_10_out.MAR_LINK_IN_END_10
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.j_marker  &
   object_value = .CHAIN_SIMPLE.ground.MAR_GROUND
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.constraint  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t1_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t2_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t3_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r1_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r2_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r3_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t1_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t2_func  &
   string_value = "1 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t3_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r1_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r2_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r3_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.t3_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .CHAIN_SIMPLE.MOTION_1.r3_ic_velo  &
   real_value = 0.0
!
ude modify instance  &
   instance_name = .CHAIN_SIMPLE.MOTION_1
