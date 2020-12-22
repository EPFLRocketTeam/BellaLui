#include "misc/state_machine_helpers.h"

#include "gtest/gtest.h"
#include "misc/rocket_constants.h"

TEST(StateMachineTests, ShouldStayInLiftoffStateDuringMotorBurnTime){
    //ASSERT_EQ(handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME - 10, 0), STATE_LIFTOFF);
    ASSERT_EQ(state_machine_helpers::handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME - 10, 0), false);
}

TEST(StateMachineTests, ShouldSwitchFromLiftoffToCoastStateAfterMotorBurnTime){
    //ASSERT_EQ(handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME + 10, 0), STATE_COAST);
    ASSERT_EQ(state_machine_helpers::handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME + 10, 0), true);
}
