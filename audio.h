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
        ma_uint32 numChannels;
        ma_uint32 sampleRate;
        ma_format format;

        Format(ma_uint32 numChannels = 0, ma_uint32 sampleRate = 0, ma_format format = ma_format_f32)
            : numChannels(numChannels)
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
            ma_decoder_config config = ma_decoder_config_init(format.format, format.numChannels, format.sampleRate);
            return ma_decoder_init_file(filePath, &config, miniaudioInstance());
        }

        ma_result Init(const void* data, size_t dataSize, Format format = Format())
        {
            ma_decoder_config config = ma_decoder_config_init(format.format, format.numChannels, format.sampleRate);
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

        ma_result Decode(std::vector<float>& samples, size_t fromFrame = 0, size_t numFrames = 0)
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
                size_t framesToRead = numFrames > 0
                    ? std::min(framesPerBuffer, numFrames - totalFramesRead)
                    : std::numeric_limits<size_t>::max();

                ma_result result = Read(buffer, framesToRead, &framesRead);
                if (result == MA_SUCCESS || result == MA_AT_END) 
                {
                    totalFramesRead += framesRead;

                    samples.insert(samples.end(), buffer, buffer + framesRead);
                    if (result == MA_AT_END || totalFramesRead == numFrames)
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
        NodeBase(ma_node_graph* graph, size_t numInputBus = 0, size_t numOutputBus = 0, ma_uint32 flags = 0)
            : _VTable
            {
                NodeBase::ProcessCallback,
                NodeBase::GetRequiredInputFrameCountCallback,
                0, // numInputBus
                0, // numOutputBus
                0  // flags
            }
        {
            _Proxy.thisNode = this;
        }

        ma_result Init(ma_node_graph* graph, ma_uint8 numInputBus = 0, ma_uint8 numOutputBus = 0, ma_uint32 flags = 0)
        {
            _VTable.inputBusCount = numInputBus;
            _VTable.outputBusCount = numOutputBus;
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

    private:
        ma_node_vtable _VTable;
        ma_node_graph* _Graph;
        struct Proxy
        {
            ma_node_base base;
            void* thisNode;
        } _Proxy;

        static NodeBase* CastToThis(ma_node* node)
        {
            return static_cast<NodeBase*>(reinterpret_cast<NodeBase::Proxy*>(node)->thisNode);
        }

        static void ProcessCallback(ma_node* node, const float** inputBuffers, ma_uint32* numInputFrames, float** outputBuffers, ma_uint32* numOutputFrames)
        {
            if (NodeBase* nodeBase = CastToThis(node))
                nodeBase->Process(inputBuffers, numInputFrames, outputBuffers, numOutputFrames);
        }

        static ma_result GetRequiredInputFrameCountCallback(ma_node* node, ma_uint32 numOutputFrames, ma_uint32* numInputFrames)
        {
            if (NodeBase* nodeBase = CastToThis(node))
                return nodeBase->GetRequiredInputFrameCount(numOutputFrames, numInputFrames);
            return MA_INVALID_DATA;
        }

    protected:
        virtual void Process(const float** inputBuffers, ma_uint32* numInputFrames, float** outputBuffers, ma_uint32* numOutputFrames) = 0;
        virtual ma_result GetRequiredInputFrameCount(ma_uint32 numOutputFrames, ma_uint32* numInputFrames) = 0;
    };
    using NodePtr = std::shared_ptr<NodeBase>;

    class DataSourceBase
    {
        ma_data_source_vtable _VTable;

    public:
        DataSourceBase(ma_uint32 flags = 0)
            : _VTable
            {
                DataSourceBase::ReadCallback,
                DataSourceBase::SeekCallback,
                DataSourceBase::GetDataFormatCallback,
                DataSourceBase::GetCursorCallback,
                DataSourceBase::GetLengthCallback,
                DataSourceBase::SetLoopingCallback,
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

    private:
        struct Proxy
        {
            ma_data_source_base base;
            void* thisDataSource;
        } _Proxy;

        static DataSourceBase* CastToThis(ma_data_source* dataSource)
        {
            return static_cast<DataSourceBase*>(reinterpret_cast<Proxy*>(dataSource)->thisDataSource);
        }

        static ma_result ReadCallback(ma_data_source* dataSource, void* buffer, ma_uint64 numFrames, ma_uint64* framesRead)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->Read(buffer, numFrames, framesRead);
            return MA_INVALID_DATA;
        }

        static ma_result SeekCallback(ma_data_source* dataSource, ma_uint64 frameIndex)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->Seek(frameIndex);
            return MA_INVALID_DATA;
        }

        static ma_result GetDataFormatCallback(ma_data_source* dataSource, ma_format* format, ma_uint32* numChannels, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetDataFormat(format, numChannels, sampleRate, channelMap, channelMapCap);
            return MA_INVALID_DATA;
        }

        static ma_result GetCursorCallback(ma_data_source* dataSource, ma_uint64* cursor)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetCursor(cursor);
            *cursor = 0;
            return MA_INVALID_DATA;
        }

        static ma_result GetLengthCallback(ma_data_source* dataSource, ma_uint64* length)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->GetLength(length);
            *length = 0;
            return MA_INVALID_DATA;
        }

        static ma_result SetLoopingCallback(ma_data_source* dataSource, ma_bool32 isLooping)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource))
                return dataSourceBase->SetLooping(isLooping);
            return MA_INVALID_DATA;
        }

    protected:
        virtual ma_result Read(void* buffer, ma_uint64 numFrames, ma_uint64* framesRead) = 0;
        virtual ma_result Seek(ma_uint64 frameIndex) = 0;
        virtual ma_result GetDataFormat(ma_format* format, ma_uint32* numChannels, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap) = 0;
        virtual ma_result GetCursor(ma_uint64* cursor) = 0;
        virtual ma_result GetLength(ma_uint64* length) = 0;
        virtual ma_result SetLooping(ma_bool32 isLooping) = 0;
    };
    using DataSourcePtr = std::shared_ptr<DataSourceBase>;

    class DataSourceNode
    {
    public:
        DataSourceNode()
            : _DataSource(nullptr)
        {
        }

        ma_result Init(DataSourcePtr dataSource, ma_node_graph* graph)
        {
            if (!dataSource || !graph)
                return MA_ERROR;

            _DataSource = dataSource;

            ma_data_source_node_config config = ma_data_source_node_config_init(_DataSource->miniaudioInstance());
            return ma_data_source_node_init(graph, &config, nullptr, miniaudioInstance());
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
        DataSourcePtr _DataSource;
        ma_data_source_node _DataSourceNode;
    };
    using DataSourceNodePtr = std::shared_ptr<DataSourceNode>;

    class Graph
    {
    public:
        Graph(ma_uint32 numChannels = 1)
            : _Graph()
        {
            ma_node_graph_config config = ma_node_graph_config_init(numChannels);
            ma_result result = ma_node_graph_init(&config, nullptr, miniaudioInstance());
            UNUSED(result);
        }

        ma_result Read(void* renderBuffer, size_t numFrames, size_t* framesRead)
        {
            return ma_node_graph_read_pcm_frames(miniaudioInstance(), renderBuffer, numFrames, framesRead);
        }

        ma_node_graph* miniaudioInstance()
        {
            return &_Graph;
        }

        ma_result AddDataSource(DataSourcePtr dataSource)
        {
            DataSourceNodePtr dataSourceNode = AddDataSourceNode(dataSource);
            if (!dataSourceNode)
                return MA_ERROR;

            return ma_node_attach_output_bus(dataSourceNode->miniaudioInstance(), 0, ma_node_graph_get_endpoint(miniaudioInstance()), 0);
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

    private:
        std::set<DataSourceNodePtr> _DataSourceNodes;
        ma_node_graph _Graph;
    };

    class DeviceBase
    {
    public:
        ma_result Init(ma_device_type deviceType = ma_device_type_playback, ma_uint32 numChannels = 0, ma_uint32 sampleRate = 0, ma_format format = ma_format_f32)
        {
            ma_device_config config = ma_device_config_init(deviceType);
            switch (deviceType)
            {
            case ma_device_type_playback:
                config.playback.format = format;
                config.playback.channels = numChannels;
                break;
            case ma_device_type_capture:
            case ma_device_type_loopback:
                config.capture.format = format;
                config.capture.channels = numChannels;
                break;
            case ma_device_type_duplex:
                config.playback.format = format;
                config.playback.channels = numChannels;
                config.capture.format = format;
                config.capture.channels = numChannels;
                break;
            default:
                return MA_DEVICE_TYPE_NOT_SUPPORTED;
            }

            config.sampleRate = sampleRate;
            config.dataCallback = DeviceBase::StaticDataCallback;
            config.pUserData = this;

            return ma_device_init(NULL, &config, miniaudioInstance());
        }

        ma_result Start()
        {
            return ma_device_start(miniaudioInstance());
        }

        ma_result Stop()
        {
            return ma_device_stop(miniaudioInstance());
        }

        void Shutdown()
        {
            ma_device_uninit(miniaudioInstance());
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
                return MA_DEVICE_TYPE_NOT_SUPPORTED;
            }
        }

        ma_device* miniaudioInstance()
        {
            return &_Device;
        }

        virtual void DataCallback(void* renderBuffer, const void* captureBuffer, ma_uint32 numFrames) = 0;

    private:
        static DeviceBase* CastToThis(ma_device* device)
        {
            return reinterpret_cast<DeviceBase*>(device->pUserData);
        }

        static void StaticDataCallback(ma_device* device, void* renderBuffer, const void* captureBuffer, ma_uint32 numFrames)
        {
            if (DeviceBase* deviceBase = CastToThis(device))
                deviceBase->DataCallback(renderBuffer, captureBuffer, numFrames);
        }

        // VDM : first implementation attempt
        // void DeviceDataCallback(void* renderBuffer, size_t numFrames)
        // {
        //     size_t framesRead;
        //     ma_result result = _Graph.Read(renderBuffer, numFrames, &framesRead);
        //     if (result != MA_SUCCESS)
        //     {
        //         ma_zero_memory_default(renderBuffer, numFrames * FrameSize());
        //         return;
        //     }
        //     if (framesRead < numFrames)
        //     {
        //         size_t bytesToZero = (numFrames - framesRead) * FrameSize();
        //         size_t offset = framesRead * FrameSize();
        //         renderBuffer = static_cast<void*>(static_cast<unsigned char*>(renderBuffer) + offset);
        //         ma_zero_memory_default(renderBuffer, bytesToZero);
        //     }
        // }
        // Graph _Graph;

    private:
        ma_device _Device;
    };

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

        const size_t GetNumFrames() const
        {
            if (_Format.numChannels == 0)
                return 0;
            return _Samples.size() / _Format.numChannels;
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
        ma_result Read(void* buffer, ma_uint64 numFrames, ma_uint64* framesRead) override
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
            const size_t numChannels = _CurrentData->GetFormat().numChannels;
            const size_t totalFrames = _CurrentData->GetNumFrames();
            float* outBuffer = static_cast<float*>(buffer);

            size_t framesToRead = numFrames;
            *framesRead = 0;

            while (framesToRead > 0)
            {
                size_t currentFrame = _Cursor % totalFrames;
                size_t framesAvailable = totalFrames - currentFrame;
                size_t framesToCopy = std::min(framesAvailable, framesToRead);

                std::memcpy(outBuffer, samples.data() + currentFrame * numChannels, framesToCopy * numChannels * sizeof(float));

                *framesRead += framesToCopy;
                framesToRead -= framesToCopy;
                outBuffer += framesToCopy * numChannels;
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

        ma_result GetDataFormat(ma_format* format, ma_uint32* numChannels, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap) override
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
            *length = _CurrentData ? _CurrentData->GetNumFrames() : 0;
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
