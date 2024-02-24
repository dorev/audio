#include "gtest/src/gtest-all.cc"

#define MACPP_ENABLED_EXTENDED_METHODS
#include "miniaudio.hpp"

#pragma region Tests helpers

#include <chrono>
#include <thread>

void WaitMs(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

static const char* DefaultTestFilePath = "beat.mp3";

#pragma endregion

namespace MiniaudioCpp
{

/**********************************************************************************************************************

Sound tests

**********************************************************************************************************************/

TEST(Sound, PlayASound)
{
    Engine engine;
    engine.Init();

    Sound sound;
    sound.InitFromFile(engine, DefaultTestFilePath);

    ma_result result = sound.Start();
    EXPECT_EQ(result, MA_SUCCESS);
    EXPECT_TRUE(sound.IsPlaying());

    while (sound.IsPlaying()) {
        WaitMs(100);
    }

    EXPECT_TRUE(sound.AtEnd());
}

TEST(Sound, StopASound)
{
    Engine engine;
    engine.Init();

    Sound sound;
    sound.InitFromFile(engine, DefaultTestFilePath);

    ma_result result = sound.Start();
    EXPECT_EQ(result, MA_SUCCESS);
    EXPECT_TRUE(sound.IsPlaying());

    WaitMs(500);
    result = sound.Stop();

    EXPECT_EQ(result, MA_SUCCESS);
    EXPECT_FALSE(sound.AtEnd());
}

/**********************************************************************************************************************

Decoder tests

**********************************************************************************************************************/

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