#include "gtest/src/gtest-all.cc"

#include "miniaudio.hpp"

namespace MiniaudioCpp
{

TEST(Engine, PlayASound)
{
    Engine engine;
    engine.Init();

    Sound sound = CreateSound("beat.mp3");

    ma_result result = sound.PlayNow();
    if (result == MA_SUCCESS)
    {
        while (sound.IsPlaying())
            WaitMs(100);

        if (!sound.AtEnd())
            FAIL() << "Sound should be finished if Sound::IsPlaying() returns false".

        sound.Stop();
    }
    else
    {
        FAIL() << "Unable to play a sound.";
    }

    // ...
}

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