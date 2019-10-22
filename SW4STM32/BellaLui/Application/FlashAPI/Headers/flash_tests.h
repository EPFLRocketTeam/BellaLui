/*
 * flash_tests.h
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */


#ifndef FLASH_TESTS_H_
#define FLASH_TESTS_H_

#include <stdint.h>



#define TEST_GENTLE_ADDR     0x00FFE000
#define TEST_INVASIVE_ADDR   0x00FFF000

#define GENTLE_TEST_BUFFER_SIZE 256

// For some arcane reason, something does not work with greater values. Reading beyond seem to be buggy.
#define INVASIVE_TEST_BUFFER_SIZE (2048 + 1024 + 512 + 256)




#define MIN (1LL) << 63


#define TEST_NOT_PERFORMED    	0
#define TEST_RUNNING	     	1
#define TEST_FINISHED		    2
#define TEST_FAILED			    -1


#define SCORE_CORE_FEATURES	 	~(MIN >> 55) // 8 most critical features
#define SCORE_BASIC_FEATURES	~(MIN >> 46) // 16 most important features
#define SCORE_LEGACY_FEATURES	~(MIN >> 31) // all legacy features
#define SCORE_EXT_FEATURES	 	~(MIN >> 15) // all legacy and extension features
#define SCORE_ALL_FEATURES	    (-1LL)



#define ALL_FEATURES 			(-1LL)


#define FEATURE_INIT 			(1L << 0)
#define FEATURE_READ			(1L << 1)
#define FEATURE_WRITE			(1L << 2)
#define FEATURE_ERASE			(1L << 3)

#define FEATURE_FS_INIT			(1L << 8)
#define FEATURE_FS_READ_BK		(1L << 9)
#define FEATURE_FS_WRITE_BK		(1L << 10)
#define FEATURE_FS_ERASE_BK		(1L << 11)
#define FEATURE_FS_TRANSFER		(1L << 12)
#define FEATURE_FS_RECOVER		(1L << 13)

#define FEATURE_LOG_INFO		(1L << 32)
#define FEATURE_LOG_WARN		(1L << 33)
#define FEATURE_LOG_ERROR		(1L << 34)
#define FEATURE_LOG_CRITICAL	(1L << 35)

/*
 * Struct that contains information about the result of a test.
 *
 * The status field should change accordingly during the test.
 * Possible values: TEST_NOT_PERFORMED, TEST_RUNNING, TEST_FINISHED, TEST_FAILED
 *
 * The score field contains information about how well the test was passed.
 * Each feature should have a bit assigned, the most critical one being
 * assigned to the LSB. Unused bits must be set to 1.
 */
typedef struct test_result {
	uint8_t status;
	uint64_t score;
} test_result;



test_result test_features(uint8_t features);
test_result benchmark_features(uint8_t features);
test_result profile_features(uint8_t features);

/*
 * Ensures that a test result is compliant with a certain standard.
 * The standard argument may be user defined or chosen from:
 * 		SCORE_CORE_FEATURES
 * 		SCORE_BASIC_FEATURES
 * 		SCORE_LEGACY_FEATURES
 * 		SCORE_EXT_FEATURES
 * 		SCORE_ALL_FEATURES
 */
uint8_t is_test_result_satisfying(test_result result, uint64_t standard);

#endif /* FLASH_TESTS_H_ */
