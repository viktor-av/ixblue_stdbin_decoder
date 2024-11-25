#ifndef TEST_STD_BIN_DECODER_ERROR_RECOVERY_FEATURE_H
#define TEST_STD_BIN_DECODER_ERROR_RECOVERY_FEATURE_H

#include <gtest/gtest.h>
#include <ixblue_stdbin_decoder/stdbin_decoder.h>

// add data for error recovery
#include "../datasets/errors/BadCheckSum.h"
#include "../datasets/errors/BadNavProtocolVersion.h"
#include "../datasets/errors/BadAnswerProtocolVersion.h"

#include "../datasets/MinimalV2NavFrame.h"
#include "../datasets/MinimalV3AnsFrame.h"

class TestStdBinDecoderErrorRecovery : public ixblue_stdbin_decoder::StdBinDecoder , public testing::Test {
    /**
     * @brief used to assert if error recovery was successfully applied
     */
    void testNavFrameErrorRecovery(){
        std::size_t parsed = 0;
        ASSERT_NO_THROW(parseNextFrame(MINIMAL_V2_NAV_FRAME.data(),MINIMAL_V2_NAV_FRAME.size(),
        		parsed)) << "Error recovery don't work.";
        ASSERT_EQ(parsed,MINIMAL_V2_NAV_FRAME.size());
    }

    /**
     * @brief used to assert if error recovery was successfully applied
     */
    void testAnswerFrameErrorRecovery(){
        //this->addNewDataFrame(MINIMAL_V3_ANS_FRAME);
        std::size_t parsed = 0;
        ASSERT_NO_THROW(parseNextFrame(MINIMAL_V3_ANS_FRAME.data(),MINIMAL_V3_ANS_FRAME.size(),
        		parsed)) << "Error recovery don't work.";
        ASSERT_EQ(parsed,MINIMAL_V3_ANS_FRAME.size());
    }
public:
    /**
     * @brief used to assert if is error is thrown
     */
    void testIsErrorIsThrown(const uint8_t* data, const std::size_t length,
    		std::size_t& parsed, ssize_t cleaned_buffer_size){
        ASSERT_THROW(parseNextFrame(data,length,parsed),std::runtime_error) << "Exception must be thrown in miss formatted data frame.";
        EXPECT_EQ(length-parsed,cleaned_buffer_size) << "Buffer must be cleaned.";
    }

    /**
     * @brief use multiple frame to test error recovery.
     * Add Nav data frame and parse it.
     * Add Answer data frame and parse it.
     */
    void testFramesErrorRecovery(){
        this->testNavFrameErrorRecovery();
        this->testAnswerFrameErrorRecovery();
    }

};
#endif //TEST_STD_BIN_DECODER_ERROR_RECOVERY_FEATURE_H
