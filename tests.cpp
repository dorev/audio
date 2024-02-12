#include "gtest/gtest.h"
#include "audio.h"

class Tests : public ::testing::Test
{
};


namespace Audio
{
    class AudioInstance
    {

    };
    using AudioInstancePtr = std::shared_ptr<AudioInstance>;

    // VDM: THIS IS ALREADY WHAT ma_engine IS DOING!!
    class EasyAPI
    {
    public:
        AudioInstancePtr Play(const char* filePath)
        {
            ma_result result = _System.Init();
            if (result != MA_SUCCESS)
                return nullptr;

            AssetPtr asset = _System.FindAsset(filePath);
            if (asset == nullptr)
            {
                asset = std::make_shared<Asset>(filePath);
                Format format = _System.GetDefaultOutputDeviceFormat();
                result = asset->Init(format.numChannels, format.sampleRate);
                if (result != MA_SUCCESS)
                    return nullptr;

                result = asset->Load();
                if (result != MA_SUCCESS)
                    return nullptr;

                _System.AddAsset(asset);
            }

            return _System.Play(asset);
            // asset keeps track of its instances
            // system creates the PCMDataSource and adds it to the graph
        };
        static System _System;

    };

}

TEST_F(Tests, Yay)
{
    // decode a file
    // start the file
    // loop the file
    // stop the file
}


