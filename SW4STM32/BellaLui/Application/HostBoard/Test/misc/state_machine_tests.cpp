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

TEST(StateMachineTests, ShouldGetCorrectImuStatus){
    uint8_t imuIsReady = state_machine_helpers::newImuDataIsAvailable(4, 3);
    EXPECT_EQ(imuIsReady, 1);
    imuIsReady = state_machine_helpers::newImuDataIsAvailable(3,3);
    EXPECT_EQ(imuIsReady, 0);
    imuIsReady = state_machine_helpers::newImuDataIsAvailable(2,3);
    EXPECT_EQ(imuIsReady, 0);
}

TEST(StateMachineTests, ShouldGetCorrectBarometerStatus){
    uint8_t barometerIsReady = state_machine_helpers::newBarometerDataIsAvailable(4, 3);
    EXPECT_EQ(barometerIsReady, 1);
    barometerIsReady = state_machine_helpers::newBarometerDataIsAvailable(3,3);
    EXPECT_EQ(barometerIsReady, 0);
    barometerIsReady = state_machine_helpers::newBarometerDataIsAvailable(2,3);
    EXPECT_EQ(barometerIsReady, 0);
}

TEST(StateMachineTests, ShouldReachTouchdownStateAfterFiveMinutes){
    uint32_t currentTime = 2*60*1000;
    uint32_t liftoffTime = 1*60*1000;
    EXPECT_FALSE(state_machine_helpers::touchdownStateIsReached(currentTime, liftoffTime));
    currentTime = 8*60*1000;
    EXPECT_TRUE(state_machine_helpers::touchdownStateIsReached(currentTime, liftoffTime));
    liftoffTime = 0;
    EXPECT_FALSE(state_machine_helpers::touchdownStateIsReached(currentTime, liftoffTime));
}

TEST(StateMachineTests, ShouldGetCorrectIdleStateStatus){
    uint32_t currentTime = 2*60*1000;
    uint32_t liftoffTime = 1*60*1000;
    uint8_t liftoffAccelTrig = 0;

    uint8_t state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, liftoffAccelTrig);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_false_positive);

    liftoffAccelTrig = 1;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, liftoffAccelTrig);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_switch_to_liftoff_state);

    currentTime = liftoffTime + 300;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, liftoffAccelTrig);
    EXPECT_EQ(state_idle_status, 0);

    liftoffTime = 0;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, liftoffAccelTrig);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_liftoff_detected);
}