#pragma once

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include <vector>
#include <set>
#include <algorithm>
#include <memory>
#include <mutex>

#define CHECK_RETURN(result) if ((result) != MA_SUCCESS) { return (result); }
#define UNUSED(variable) (void)variable

#if defined(_MSC_VER)
    #define AUDIO_DEBUG_BREAK() __debugbreak()
    #define AUDIO_FUNCTION __FUNCSIG__
#elif defined(__GNUC__) || defined(__clang__)
    #define AUDIO_DEBUG_BREAK() __builtin_trap()
    #define AUDIO_FUNCTION __PRETTY_FUNCTION__
#else
    #define AUDIO_DEBUG_BREAK() assert(false)
    #define AUDIO_FUNCTION __FUNCTION__
#endif

#define AUDIO_LOG(format, ...) printf("[Audio] " format "\n", ##__VA_ARGS__)
#define AUDIO_LOG_WARNING(format, ...) printf("[Audio] [WARNING] " format "  {%s}\n", AUDIO_FUNCTION, ##__VA_ARGS__)
#define AUDIO_LOG_ERROR(format, ...) printf("[Audio] [ERROR] " format "  {%s, %s l.%d}\n", AUDIO_FUNCTION, ##__VA_ARGS__, __FILE__, __LINE__)

#define AUDIO_DEBUG_ASSERT(condition, format, ...) \
if (!(condition)) \
{ \
    AUDIO_LOG("[ASSERT] " format "\n", ##__VA_ARGS__); \
    AUDIO_DEBUG_BREAK(); \
}

namespace Audio
{
    struct Format
    {
        ma_uint32 channelCount;
        ma_uint32 sampleRate;
        ma_format format;

        Format(ma_uint32 channelCount = 0, ma_uint32 sampleRate = 0, ma_format format = ma_format_f32)
            : channelCount(channelCount)
            , sampleRate(sampleRate)
            , format(format)
        {
        }
    };

    class Decoder
    {
    public:
        ~Decoder()
        {
            ma_decoder_uninit(miniaudioInstance());
        }

        ma_result Init(const char* filePath, Format format = Format())
        {
            ma_decoder_config config = ma_decoder_config_init(format.format, format.channelCount, format.sampleRate);
            return ma_decoder_init_file(filePath, &config, miniaudioInstance());
        }

        ma_result Init(const void* data, size_t dataSize, Format format = Format())
        {
            ma_decoder_config config = ma_decoder_config_init(format.format, format.channelCount, format.sampleRate);
            return ma_decoder_init_memory(data, dataSize, &config, miniaudioInstance());
        }

        Format GetFormat() const
        {
            return Format(_Decoder.outputChannels, _Decoder.outputSampleRate, _Decoder.outputFormat);
        }

        ma_result Read(void* buffer, size_t frameCount, size_t* framesRead)
        {
            return ma_decoder_read_pcm_frames(miniaudioInstance(), buffer, frameCount, framesRead);
        };

        ma_result Seek(size_t targetFrame)
        {
            return ma_decoder_seek_to_pcm_frame(miniaudioInstance(), targetFrame);
        }
        
        ma_result GetAvailableFrames(size_t* availableFrames)
        {
            return ma_decoder_get_available_frames(miniaudioInstance(), availableFrames);
        }

        ma_result GetCursor(size_t* cursor)
        {
            return ma_decoder_get_cursor_in_pcm_frames(miniaudioInstance(), cursor);
        }

        ma_result GetLength(size_t* length)
        {
            return ma_decoder_get_length_in_pcm_frames(miniaudioInstance(), length);
        }

        ma_result GetAvailableFrames(size_t* availableFrames)
        {
            return ma_decoder_get_available_frames(miniaudioInstance(), availableFrames);
        }

        ma_decoder* miniaudioInstance()
        {
            return &_Decoder;
        }

        ma_result Decode(std::vector<float>& samples, size_t fromFrame = 0, size_t frameCount = 0)
        {
            if (fromFrame > 0)
                CHECK_RETURN(Seek(fromFrame));

            samples.clear();

            constexpr size_t kDecoderBufferSize = 4096;
            float buffer[kDecoderBufferSize];
            size_t framesPerBuffer = sizeof(buffer) / sizeof(buffer[0]);
            size_t totalFramesRead = 0;
            size_t framesRead = 0;
            while (true)
            {
                size_t framesToRead = frameCount > 0
                    ? std::min(framesPerBuffer, frameCount - totalFramesRead)
                    : std::numeric_limits<size_t>::max();

                ma_result result = Read(buffer, framesToRead, &framesRead);
                if (result == MA_SUCCESS || result == MA_AT_END) 
                {
                    totalFramesRead += framesRead;

                    samples.insert(samples.end(), buffer, buffer + framesRead);
                    if (result == MA_AT_END || totalFramesRead == frameCount)
                        return MA_SUCCESS;
                }
                else
                {
                    return result;
                }
            }
        }

    private:
        ma_decoder _Decoder;
    };

    class NodeBase
    {
    public:
        NodeBase(ma_node_graph* graph, size_t inputBusCount = 0, size_t outputBusCount = 0, ma_uint32 flags = 0)
            : _VTable
            {
                NodeBase::ProcessCallback,
                NodeBase::GetRequiredInputFrameCountCallback,
                0, // inputBusCount
                0, // outputBusCount
                0  // flags
            }
        {
            _Proxy.thisNode = this;
        }

        ma_result Init(ma_node_graph* graph, ma_uint8 inputBusCount = 0, ma_uint8 outputBusCount = 0, ma_uint32 flags = 0)
        {
            _VTable.inputBusCount = inputBusCount;
            _VTable.outputBusCount = outputBusCount;
            _VTable.flags = flags;

            ma_node_config config = ma_node_config_init();
            config.vtable = &_VTable;
            return ma_node_init(graph, &config, nullptr, miniaudioInstance());
        }

        virtual ~NodeBase()
        {
            ma_node_uninit(miniaudioInstance(), nullptr);
        }

        ma_node* miniaudioInstance()
        {
            return reinterpret_cast<ma_node*>(&_Proxy);
        }

    protected:
        virtual void Process(const float** inputBuffers, ma_uint32* inputFrameCount, float** outputBuffers, ma_uint32* outputFrameCount) = 0;
        virtual ma_result GetRequiredInputFrameCount(ma_uint32 outputFrameCount, ma_uint32* inputFrameCount) = 0;

    private:
        static NodeBase* CastToThis(ma_node* node)
        {
            return static_cast<NodeBase*>(reinterpret_cast<NodeBase::Proxy*>(node)->thisNode);
        }

        static void ProcessCallback(ma_node* node, const float** inputBuffers, ma_uint32* inputFrameCount, float** outputBuffers, ma_uint32* outputFrameCount)
        {
            if (NodeBase* nodeBase = CastToThis(node))
                nodeBase->Process(inputBuffers, inputFrameCount, outputBuffers, outputFrameCount);
        }

        static ma_result GetRequiredInputFrameCountCallback(ma_node* node, ma_uint32 outputFrameCount, ma_uint32* inputFrameCount)
        {
            if (NodeBase* nodeBase = CastToThis(node))
                return nodeBase->GetRequiredInputFrameCount(outputFrameCount, inputFrameCount);
            return MA_INVALID_DATA;
        }
    
    private:
        ma_node_vtable _VTable;
        ma_node_graph* _Graph;
        struct Proxy
        {
            ma_node_base base;
            void* thisNode;
        } _Proxy;
    };
    using NodePtr = std::shared_ptr<NodeBase>;

    class DataSourceBase
    {
    public:
        DataSourceBase(ma_uint32 flags = 0)
            : _VTable
            {
                DataSourceBase::StaticRead,
                DataSourceBase::StaticSeek,
                DataSourceBase::StaticGetDataFormat,
                DataSourceBase::StaticGetCursor,
                DataSourceBase::StaticGetLength,
                DataSourceBase::StaticSetLooping,
                0 // flags
            }
        {
            _Proxy.thisDataSource = this;
        }

        ma_result Init(ma_uint32 flags = 0)
        {
            ma_data_source_config config = ma_data_source_config_init();
            _VTable.flags = flags;
            config.vtable = &_VTable;
            return ma_data_source_init(&config, miniaudioInstance());
        }

        virtual ~DataSourceBase()
        {
            ma_data_source_uninit(miniaudioInstance());
        }

        ma_data_source* miniaudioInstance()
        {
            return reinterpret_cast<ma_data_source*>(&_Proxy);
        }

    protected:
        virtual ma_result Read(void* buffer, ma_uint64 frameCount, ma_uint64* framesRead) = 0;
        virtual ma_result Seek(ma_uint64 frameIndex) = 0;
        virtual ma_result GetDataFormat(ma_format* format, ma_uint32* channelCount, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap) = 0;
        virtual ma_result GetCursor(ma_uint64* cursor) = 0;
        virtual ma_result GetLength(ma_uint64* length) = 0;
        virtual ma_result SetLooping(ma_bool32 isLooping) = 0;

    private:
        static DataSourceBase* CastToThis(ma_data_source* dataSource)
        {
            return static_cast<DataSourceBase*>(reinterpret_cast<Proxy*>(dataSource)->thisDataSource);
        }

        static ma_result StaticRead(ma_data_source* dataSource, void* buffer, ma_uint64 frameCount, ma_uint64* framesRead)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->Read(buffer, frameCount, framesRead);
            return MA_INVALID_DATA;
        }

        static ma_result StaticSeek(ma_data_source* dataSource, ma_uint64 frameIndex)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->Seek(frameIndex);
            return MA_INVALID_DATA;
        }

        static ma_result StaticGetDataFormat(ma_data_source* dataSource, ma_format* format, ma_uint32* channelCount, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetDataFormat(format, channelCount, sampleRate, channelMap, channelMapCap);
            return MA_INVALID_DATA;
        }

        static ma_result StaticGetCursor(ma_data_source* dataSource, ma_uint64* cursor)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetCursor(cursor);
            *cursor = 0;
            return MA_INVALID_DATA;
        }

        static ma_result StaticGetLength(ma_data_source* dataSource, ma_uint64* length)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetLength(length);
            *length = 0;
            return MA_INVALID_DATA;
        }

        static ma_result StaticSetLooping(ma_data_source* dataSource, ma_bool32 isLooping)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->SetLooping(isLooping);
            return MA_INVALID_DATA;
        }

    private:
        ma_data_source_vtable _VTable;
        struct Proxy
        {
            ma_data_source_base base;
            void* thisDataSource;
        } _Proxy;
    };
    using DataSourcePtr = std::shared_ptr<DataSourceBase>;

    class DataSourceNode
    {
    public:
        DataSourceNode()
        {
        }

        ma_result Init(ma_data_source* dataSource, ma_node_graph* graph, ma_allocation_callbacks* allocationCallbacks = nullptr)
        {
            ma_data_source_node_config config = ma_data_source_node_config_init(dataSource);
            return ma_data_source_node_init(graph, &config, allocationCallbacks, miniaudioInstance());
        }

        ~DataSourceNode()
        {
            ma_data_source_node_uninit(miniaudioInstance(), nullptr);
        }

        ma_data_source_node* miniaudioInstance()
        {
            return &_DataSourceNode;
        }

    private:
        ma_data_source_node _DataSourceNode;
    };
    using DataSourceNodePtr = std::shared_ptr<DataSourceNode>;

    class Graph
    {
    public:
        Graph()
            : _Graph()
        {
        }

        ma_result Init(ma_uint32 channelCount = 1)
        {
            ma_node_graph_config config = ma_node_graph_config_init(channelCount);
            return ma_node_graph_init(&config, nullptr, miniaudioInstance());
        }

        ma_result Read(void* renderBuffer, size_t frameCount, size_t* framesRead)
        {
            return ma_node_graph_read_pcm_frames(miniaudioInstance(), renderBuffer, frameCount, framesRead);
        }

        ma_result DetachNodeFull(ma_node* node)
        {
            return ma_node_detach_full(node);
        }

        ma_result AttachToEndpoint(ma_node* node)
        {
            return ma_node_attach_output_bus(node, 0,  ma_node_graph_get_endpoint(miniaudioInstance()), 0);
        }

        ma_node_graph* miniaudioInstance()
        {
            return &_Graph;
        }
/*
        ma_result AddDataSource(DataSourcePtr dataSource)
        {
            DataSourceNodePtr dataSourceNode = AddDataSourceNode(dataSource);
            if (!dataSourceNode)
                return MA_ERROR;

            return AttachToEndpoint(dataSourceNode->miniaudioInstance());
        }

        ma_result RemoveDataSourceNode(DataSourceNodePtr dataSourceNode)
        {
            return ma_node_detach_full(dataSourceNode ? dataSourceNode->miniaudioInstance() : nullptr);
        }

    private:
        DataSourceNodePtr AddDataSourceNode(DataSourcePtr dataSource)
        {
            DataSourceNodePtr dataSourceNode = std::make_shared<DataSourceNode>();
            dataSourceNode->Init(dataSource, miniaudioInstance());
            auto itrBool = _DataSourceNodes.insert(dataSourceNode);
            return itrBool.second ? *itrBool.first : nullptr;
        }
        std::set<DataSourceNodePtr> _DataSourceNodes;
*/

    private:
        ma_node_graph _Graph;
    };

    class DeviceBase
    {
    public:
        ma_result Init(ma_device_type deviceType = ma_device_type_playback, ma_uint32 channelCount = 0, ma_uint32 sampleRate = 0, ma_format format = ma_format_f32, ma_context* context = nullptr)
        {
            ma_device_config config = ma_device_config_init(deviceType);
            switch (deviceType)
            {
            case ma_device_type_playback:
                config.playback.format = format;
                config.playback.channels = channelCount;
                break;
            case ma_device_type_capture:
            case ma_device_type_loopback:
                config.capture.format = format;
                config.capture.channels = channelCount;
                break;
            case ma_device_type_duplex:
                config.playback.format = format;
                config.playback.channels = channelCount;
                config.capture.format = format;
                config.capture.channels = channelCount;
                break;
            default:
                return MA_DEVICE_TYPE_NOT_SUPPORTED;
            }

            config.sampleRate = sampleRate;
            config.dataCallback = DeviceBase::StaticDataCallback;
            config.pUserData = this;

            return ma_device_init(context, &config, miniaudioInstance());
        }

        virtual ~DeviceBase()
        {
            ma_device_uninit(miniaudioInstance());
        }

        ma_result Start()
        {
            return ma_device_start(miniaudioInstance());
        }

        ma_result Stop()
        {
            return ma_device_stop(miniaudioInstance());
        }

        size_t FrameSize(ma_device_type deviceType = ma_device_type_playback) const
        {
            switch (deviceType)
            {
            case ma_device_type_playback:
                return ma_get_bytes_per_frame(_Device.playback.format, _Device.playback.channels);

            case ma_device_type_capture:
            case ma_device_type_loopback:
                return ma_get_bytes_per_frame(_Device.capture.format, _Device.capture.channels);

            case ma_device_type_duplex:
            default:
                return 0;
            }
        }

        ma_device* miniaudioInstance()
        {
            return &_Device;
        }

        virtual void DataCallback(void* renderBuffer, const void* captureBuffer, ma_uint32 frameCount) = 0;

    private:
        static DeviceBase* CastToThis(ma_device* device)
        {
            return reinterpret_cast<DeviceBase*>(device->pUserData);
        }

        static void StaticDataCallback(ma_device* device, void* renderBuffer, const void* captureBuffer, ma_uint32 frameCount)
        {
            if (DeviceBase* deviceBase = CastToThis(device))
                deviceBase->DataCallback(renderBuffer, captureBuffer, frameCount);
        }

        // VDM : first implementation attempt
        // void DeviceDataCallback(void* renderBuffer, size_t frameCount)
        // {
        //     size_t framesRead;
        //     ma_result result = _Graph.Read(renderBuffer, frameCount, &framesRead);
        //     if (result != MA_SUCCESS)
        //     {
        //         ma_zero_memory_default(renderBuffer, frameCount * FrameSize());
        //         return;
        //     }
        //     if (framesRead < frameCount)
        //     {
        //         size_t bytesToZero = (frameCount - framesRead) * FrameSize();
        //         size_t offset = framesRead * FrameSize();
        //         renderBuffer = static_cast<void*>(static_cast<unsigned char*>(renderBuffer) + offset);
        //         ma_zero_memory_default(renderBuffer, bytesToZero);
        //     }
        // }
        // Graph _Graph;

    private:
        ma_device _Device;
    };

    class Context
    {
    public:
        Context()
        {
        }

        ma_result Init(const ma_backend backends[] = nullptr, ma_uint32 backendCount = 0, ma_allocation_callbacks allocationBallbacks = ma_allocation_callbacks_init_default())
        {
            ma_context_config config = ma_context_config_init();
            config.allocationCallbacks = allocationBallbacks;
            return ma_context_init(backends, backendCount, &config, miniaudioInstance());
        }

        ma_context* miniaudioInstance()
        {
            return &_Context;
        }

    private:
        ma_context _Context;
    };

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CODE BELOW IS NOT STRICTLY WRAPPING MINIAUDIO, JUST THINGS RELATED TO EXPERIMENTATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class PCMData
    {
    public:
        PCMData()
            : _Samples()
            , _Format()
        {
        }

        std::vector<float>& GetSamplesForWriting()
        {
            return _Samples;
        }

        const std::vector<float>& GetSamples() const
        {
            return _Samples;
        }

        const size_t GetframeCount() const
        {
            if (_Format.channelCount == 0)
                return 0;
            return _Samples.size() / _Format.channelCount;
        }

        const Format& GetFormat() const
        {
            return _Format;
        }

        void SetFormat(Format format)
        {
            _Format = format;
        }

    private:
        std::vector<float> _Samples;
        Format _Format;
    };
    using PCMDataPtr = std::shared_ptr<PCMData>;

    class Asset
    {
    public:
        enum State
        {
            Unloaded,
            Loading,
            Loaded,
            Streamed
        };

        Asset(const char* filePath)
            : _FilePath(filePath)
            , _Decoder()
            , _PCMData()
        {
        }

        ma_result Init(Format format = Format())
        {
            ma_result result = _Decoder.Init(_FilePath.c_str(), format);
            CHECK_RETURN(result);
            _PCMData.SetFormat(_Decoder.GetFormat());
        }

        ma_result Load()
        {
            return _Decoder.Decode(_PCMData.GetSamplesForWriting());
        }

    private:
        std::string _FilePath;
        Decoder _Decoder;
        PCMData _PCMData;
    };
    using AssetPtr = std::shared_ptr<Asset>;

    class PCMDataSource : public DataSourceBase
    {
    public:
        PCMDataSource(PCMDataPtr pcmData = nullptr)
            : _CurrentData(pcmData)
            , _NextData(nullptr)
        {
        }

        void Reset(PCMDataPtr pcmData)
        {
            std::lock_guard lock(_NextDataMutex);
            _NextData = pcmData;
            _Cursor = 0;
        }

    private:
        ma_result Read(void* buffer, ma_uint64 frameCount, ma_uint64* framesRead) override
        {
            {
                std::lock_guard lock(_NextDataMutex);
                if (_NextData)
                    _CurrentData = _NextData;
                _NextData = nullptr;
            }

            if (_CurrentData == nullptr || (_CurrentData && _CurrentData->GetSamples().empty()))
            {
                *framesRead = 0;
                return MA_NO_DATA_AVAILABLE;
            }

            const std::vector<float>& samples = _CurrentData->GetSamples();
            const size_t channelCount = _CurrentData->GetFormat().channelCount;
            const size_t totalFrames = _CurrentData->GetframeCount();
            float* outBuffer = static_cast<float*>(buffer);

            size_t framesToRead = frameCount;
            *framesRead = 0;

            while (framesToRead > 0)
            {
                size_t currentFrame = _Cursor % totalFrames;
                size_t framesAvailable = totalFrames - currentFrame;
                size_t framesToCopy = std::min(framesAvailable, framesToRead);

                std::memcpy(outBuffer, samples.data() + currentFrame * channelCount, framesToCopy * channelCount * sizeof(float));

                *framesRead += framesToCopy;
                framesToRead -= framesToCopy;
                outBuffer += framesToCopy * channelCount;
                _Cursor += framesToCopy;

                if (!_IsLooping || framesToCopy == 0)
                    break;
            }

            if (_IsLooping && framesToRead > 0)
            {
                _Cursor = 0;
                return Read(buffer, framesToRead, framesRead);
            }

            return MA_SUCCESS;
        }

        ma_result Seek(ma_uint64 frameIndex) override
        {
            _Cursor = frameIndex;
            return MA_SUCCESS;
        }

        ma_result GetDataFormat(ma_format* format, ma_uint32* channelCount, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap) override
        {
            return MA_SUCCESS;
        }

        ma_result GetCursor(ma_uint64* cursor) override
        {
            *cursor = _Cursor;
            return MA_SUCCESS;
        }

        ma_result GetLength(ma_uint64* length) override
        {
            *length = _CurrentData ? _CurrentData->GetframeCount() : 0;
            return MA_SUCCESS;
        }

        ma_result SetLooping(ma_bool32 isLooping) override
        {
            _IsLooping = isLooping;
            return MA_SUCCESS;
        }

    private:
        size_t _Cursor;
        bool _IsLooping;
        PCMDataPtr _CurrentData;
        PCMDataPtr _NextData;
        std::mutex _NextDataMutex;
    };

    class System
    {
    public:
        ma_result Init()
        {
            if (_Devices.empty())
            {
                DeviceBase& device = *_Devices.emplace_back(std::make_unique<DeviceBase>());
                return device.Init();
            }
            return MA_SUCCESS;
        }

        ma_result LoadSound(const char* filePath)
        {
            return MA_NOT_IMPLEMENTED;
        }

    private:
        std::vector<std::unique_ptr<DeviceBase>> _Devices;
        std::vector<AssetPtr> _Assets;
    };
}
