#include "gtest/src/gtest-all.cc"

#include "miniaudio.hpp"

namespace MiniaudioCpp
{

TEST(Decoder, DryLifeCycle)
{
    Decoder decoder;
}

TEST(Decoder, FailInitWithEmptyPath)
{
    Decoder decoder;
    ma_result result = decoder.Init("");
    EXPECT_EQ(result, MA_INVALID_ARGS);
}

} // namespace MiniaudioCpp


/**********************************************************************************************************************

GoogleTest main

**********************************************************************************************************************/

int main(int argc, char **argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}