#include "misc/state_machine_helpers.h"

#include "gtest/gtest.h"
#include "misc/rocket_constants.h"

TEST(StateMachineTests, ShouldStayInLiftoffStateDuringMotorBurnTime){
    //ASSERT_EQ(handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME - 10, 0), STATE_LIFTOFF);
    ASSERT_FALSE(state_machine_helpers::handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME - 10, 0));
}

TEST(StateMachineTests, ShouldSwitchFromLiftoffToCoastStateAfterMotorBurnTime){
    //ASSERT_EQ(handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME + 10, 0), STATE_COAST);
    ASSERT_TRUE(state_machine_helpers::handleLiftoffState(ROCKET_CST_MOTOR_BURNTIME + 10, 0));
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
    liftoffTime = NO_LIFTOFF_TIME;
    EXPECT_FALSE(state_machine_helpers::touchdownStateIsReached(currentTime, liftoffTime));
}

TEST(StateMachineTests, ShouldGetCorrectIdleStateStatus){
    uint32_t currentTime = 2*60*1000;
    uint32_t liftoffTime = 1*60*1000;
    float acceleration_z = 1;

    uint8_t state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, acceleration_z);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_false_positive);

    acceleration_z = 2.1;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, acceleration_z);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_switch_to_liftoff_state);

    currentTime = liftoffTime + 300;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, acceleration_z);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_no_op);

    liftoffTime = NO_LIFTOFF_TIME;
    state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoffTime, acceleration_z);
    EXPECT_EQ(state_idle_status, state_machine_helpers::state_idle_liftoff_detected);
}

TEST(StateMachineTests, ShouldGetCorrectPrimaryStateStatus){
    // ROCKET_CST_REC_SECONDARY_ALT == 150
    // APOGEE_MUTE_TIME == 5000
    // SECONDARY_BUFFER_SIZE == 5
    uint32_t currentTime = 1*60*1000 + 60;
    uint32_t time_tmp = 1*60*1000;

    uint32_t sec_counter = 1; 

    float baro_data_base_altitude = 100;
    float baro_data_altitude = 120;

    uint8_t state_primary_status = state_machine_helpers::handlePrimaryState(currentTime,  time_tmp, baro_data_altitude, baro_data_base_altitude, sec_counter);
    EXPECT_EQ(state_primary_status, state_machine_helpers::state_primary_no_op);

    sec_counter = SECONDARY_BUFFER_SIZE;
    state_primary_status = state_machine_helpers::handlePrimaryState(currentTime,  time_tmp, baro_data_altitude, baro_data_base_altitude, sec_counter);
    EXPECT_EQ(state_primary_status, state_machine_helpers::state_primary_no_op);

    currentTime = time_tmp + APOGEE_MUTE_TIME + 1;
    state_primary_status = state_machine_helpers::handlePrimaryState(currentTime,  time_tmp, baro_data_altitude, baro_data_base_altitude, sec_counter);
    EXPECT_EQ(state_primary_status, state_machine_helpers::state_primary_switch_to_secondary_state);

    sec_counter = SECONDARY_BUFFER_SIZE - 1;
    state_primary_status = state_machine_helpers::handlePrimaryState(currentTime,  time_tmp, baro_data_altitude, baro_data_base_altitude, sec_counter);
    EXPECT_EQ(state_primary_status, state_machine_helpers::state_primary_no_op);

    sec_counter = SECONDARY_BUFFER_SIZE;

    baro_data_altitude = baro_data_base_altitude + ROCKET_CST_REC_SECONDARY_ALT + 1;
    state_primary_status = state_machine_helpers::handlePrimaryState(currentTime,  time_tmp, baro_data_altitude, baro_data_base_altitude, sec_counter); 
    EXPECT_EQ(state_primary_status, state_machine_helpers::state_primary_altitude_above_secondary_altitude);
}

TEST(StateMachineTests, ShouldGetCorrectSecondaryStateStatus){
    // TOUCHDOWN_BUFFER_SIZE == 5
    // TOUCHDOWN_DELAY_TIME == 2000
    // TOUCHDOWN_ALT_DIFF == 2
    uint32_t currentTime = 1*60*1000 + 60;
    uint32_t time_tmp = 1*60*1000;

    uint32_t sec_counter = 1; 

    bool baro_is_ready = false;
    float baro_data_altitude = 120;

    uint8_t td_counter = 1;
    float td_last_alt = 130;

    uint8_t state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_no_op);

    baro_is_ready = true;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_no_op);

    currentTime = time_tmp + TOUCHDOWN_DELAY_TIME + 10;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_altitude_difference_still_large);
    
    baro_data_altitude = td_last_alt + TOUCHDOWN_ALT_DIFF + 1;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_altitude_difference_still_large);

    baro_data_altitude = td_last_alt + TOUCHDOWN_ALT_DIFF;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_approaching_touchdown);

    td_counter = TOUCHDOWN_BUFFER_SIZE - 1;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_approaching_touchdown);

    td_counter = TOUCHDOWN_BUFFER_SIZE;
    state_secondary_status = state_machine_helpers::handleSecondaryState(currentTime, time_tmp, baro_is_ready, baro_data_altitude, td_last_alt, td_counter);
    EXPECT_EQ(state_secondary_status, state_machine_helpers::state_secondary_switch_to_touchdown_state);
}
