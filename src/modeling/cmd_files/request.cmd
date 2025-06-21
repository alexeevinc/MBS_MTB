!
output_control create request  &
   request_name = .CHAIN_TEST.CONTACT_req  &
   adams_id = 1  &
   component_names = "", "PenetrationDepth", "PenetrationVelocity",  &
                     "theNormalForce", "", "theKPortion", "theCPortion", ""  &
   component_units = "no_units", "length", "velocity", "force", "no_units",  &
                     "force", "force", "no_units"  &
   results_name = "CONTACT_data"  &
   user_function = 1.0  &
   routine = "C_REQSUB::REQSUB"
!
undo end_block