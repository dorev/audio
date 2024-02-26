#pragma once

/**********************************************************************************************************************

`miniaudio` configuration

**********************************************************************************************************************/

#define MINIAUDIO_IMPLEMENTATION

#include "miniaudio.h"

/* Post-configuration of `miniaudio` */

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

/**********************************************************************************************************************

MiniaudioCpp macros

**********************************************************************************************************************/

#define MACPP_CHECK_RESULT(result) { ma_result __result__ = result; if (__result__ != MA_SUCCESS) { return __result__; } }
#define UNUSED(variable) (void)variable

#if defined(_MSC_VER)
    #define MACPP_DEBUG_BREAK() __debugbreak()
    #define MACPP_FUNCTION __FUNCSIG__
#elif defined(__GNUC__) || defined(__clang__)
    #define MACPP_DEBUG_BREAK() __builtin_trap()
    #define MACPP_FUNCTION __PRETTY_FUNCTION__
#else
    #define MACPP_DEBUG_BREAK() assert(false)
    #define MACPP_FUNCTION __FUNCTION__
#endif

#define MACPP_LOG(format, ...) printf("[MACPP] " format "\n", ##__VA_ARGS__)
#define MACPP_LOG_WARNING(format, ...) printf("[MACPP - WARNING] " format "  {%s}\n", MACPP_FUNCTION, ##__VA_ARGS__)
#define MACPP_LOG_ERROR(format, ...) printf("[MACPP - ERROR] " format "  {%s, %s l.%d}\n", MACPP_FUNCTION, ##__VA_ARGS__, __FILE__, __LINE__)

#define MACPP_DEBUG_ASSERT(condition, format, ...) \
if (!(condition)) \
{ \
    MACPP_LOG("[ASSERT] " format "\n", ##__VA_ARGS__); \
    MACPP_DEBUG_BREAK(); \
}

/**********************************************************************************************************************

MiniaudioCpp config

**********************************************************************************************************************/

#define MACPP_VERSION_MAJOR    0
#define MACPP_VERSION_MINOR    0
#define MACPP_VERSION_REVISION 1
#define MACPP_VERSION_STRING   MA_XSTRINGIFY(MACPP_VERSION_MAJOR) "." MA_XSTRINGIFY(MACPP_VERSION_MINOR) "." MA_XSTRINGIFY(MACPP_VERSION_REVISION)

/* Extended methods refer to addition that were made on top of the `miniaudio` API. */
#if !defined(MACPP_ENABLE_EXTENDED_METHODS)
    #define MACPP_EXTENDED_METHODS_ENABLED false
#else
    #define MACPP_EXTENDED_METHODS_ENABLED true
#endif

#if MACPP_EXTENDED_METHODS_ENABLED

/*

Extended methods use the following `std` classes

* std::vector
* std::shared_ptr
* ...

Aliases are provided for these classes for simple substitution. If you intend to use different classes here, keep in mind
that they must respect the corresponding `std` API.

*/

#include <vector>
#include <memory>

namespace MiniaudioCpp
{
    template <class T>
    using Vector = std::vector<T>;

    template <class T>
    using SharedPtr = std::shared_ptr<T>;

    template <class T, class... Args>
    SharedPtr<T> MakeShared(Args&&... args)
    {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }
}

#endif // MACPP_EXTENDED_METHODS_ENABLED

namespace MiniaudioCpp
{
    template <class T>
    class MiniaudioObject
    {
    public:
        using Type = T;

        virtual ~MiniaudioObject()
        {
        };

        virtual T* GetMiniaudioObject() = 0;

        operator T*()
        {
            return GetMiniaudioObject();
        }
    };

/**********************************************************************************************************************

Decoder

**********************************************************************************************************************/

    class Decoder : public MiniaudioObject<ma_decoder>
    {
    public:
        ma_decoder* GetMiniaudioObject()
        {
            return &_Decoder;
        }

        const ma_decoder_config* GetConfig()
        {
            return &_Config;
        }

        Decoder()
        {
            memset(&_Decoder, 0, sizeof(MiniaudioObject::Type));
            memset(&_Config, 0, sizeof(ma_decoder_config));
        }

        virtual ~Decoder()
        {
            ma_decoder_uninit(&_Decoder);
        }
        
        ma_result InitFile(
            const char* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            _Config = config;
            return ma_decoder_init_file(filePath, &_Config, &_Decoder);
        }

        static ma_result DecodeFromVFS(
            ma_vfs* vfs,
            const char* filePath,
            ma_uint64* frameCountOut,
            void** pcmFramesOut,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decode_from_vfs(vfs, filePath, &config, frameCountOut, pcmFramesOut);
        }

        static ma_result DecodeFile(
            const char* filePath,
            ma_uint64* frameCountOut,
            void** pcmFramesOut,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decode_file(filePath, &config, frameCountOut, pcmFramesOut);
        }

        static ma_result DecodeMemory(
            const void* data,
            size_t dataSize,
            ma_uint64* frameCountOut,
            void** pcmFramesOut,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decode_memory(data, dataSize, &config, frameCountOut, pcmFramesOut);
        }

        ma_result InitFile(
            const wchar_t* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            _Config = config;
            return ma_decoder_init_file_w(filePath, &_Config, &_Decoder);
        }

        ma_result InitMemory(
            const void* data,
            size_t dataSize,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            _Config = config;
            return ma_decoder_init_memory(data, dataSize, &_Config, &_Decoder);
        }

        ma_result InitVFS(
            ma_vfs* vfs,
            const char* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            _Config = config;
            return ma_decoder_init_vfs(vfs, filePath, &_Config, &_Decoder);
        }

        ma_result InitVFS(
            ma_vfs* vfs,
            const wchar_t* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            _Config = config;
            return ma_decoder_init_vfs_w(vfs, filePath, &_Config, &_Decoder);
        }

        ma_result ReadPCMFrames(void* buffer, ma_uint64 frameCount, ma_uint64* framesRead)
        {
            return ma_decoder_read_pcm_frames(&_Decoder, buffer, frameCount, framesRead);
        };

        ma_result SeekToPCMFrame(ma_uint64 targetFrame)
        {
            return ma_decoder_seek_to_pcm_frame(&_Decoder, targetFrame);
        }
        
        ma_result GetAvailableFrames(ma_uint64* availableFrames)
        {
            return ma_decoder_get_available_frames(&_Decoder, availableFrames);
        }

        ma_result GetCursorInPCMFrames(ma_uint64* cursor)
        {
            return ma_decoder_get_cursor_in_pcm_frames(&_Decoder, cursor);
        }

        ma_result GetLengthInPCMFrames(ma_uint64* length)
        {
            return ma_decoder_get_length_in_pcm_frames(&_Decoder, length);
        }

        ma_result GetDataFormat(ma_format* format, ma_uint32* channels, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap)
        {
            return ma_decoder_get_data_format(&_Decoder, format, channels, sampleRate, channelMap, channelMapCap);
        }

#if MACPP_EXTENDED_METHODS_ENABLED

        ma_result Decode(Vector<float>& samples, size_t fromFrame = 0, size_t frameCount = 0)
        {
            if (fromFrame > 0)
                MACPP_CHECK_RESULT(SeekToPCMFrame(fromFrame));

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

                ma_result result = ReadPCMFrames(buffer, framesToRead, &framesRead);
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

#endif // MACPP_EXTENDED_METHODS_ENABLED

    protected:
        ma_decoder _Decoder;
        ma_decoder_config _Config;
    };

    class CustomDecoderBase : public Decoder
    {
    public:
        ma_result Init(ma_decoder_config config = ma_decoder_config_init_default())
        {
            _Config = config;
            ma_decoder_seek_proc o = 0;
            return ma_decoder_init(StaticReadCallback, StaticSeekCallback, this, GetConfig(), GetMiniaudioObject());
        }

        virtual ma_result ReadCallback(void* bufferOut, size_t bytesToRead, size_t* bytesRead) = 0;
        virtual ma_result SeekCallback(ma_uint64 byteOffset, ma_seek_origin origin) = 0;

    private:
        static ma_result StaticReadCallback(ma_decoder* decoder, void* bufferOut, size_t bytesToRead, size_t* bytesRead)
        {
            if (!decoder) {
                return MA_ERROR;
            }

            CustomDecoderBase* instance = reinterpret_cast<CustomDecoderBase*>(decoder->pUserData);

            if (!instance) {
                return MA_ERROR;
            }

            return instance->ReadCallback(bufferOut, bytesToRead, bytesRead);
        }

        static ma_result StaticSeekCallback(ma_decoder* decoder, ma_int64 byteOffset, ma_seek_origin origin)
        {
            if (!decoder) {
                return MA_ERROR;
            }

            CustomDecoderBase* instance = reinterpret_cast<CustomDecoderBase*>(decoder->pUserData);

            if (!instance) {
                return MA_ERROR;
            }

            return instance->SeekCallback(byteOffset, origin);
        }
    };

    // TODO: make a standard type wrapper, then a CustomBase derived class
    class NodeBase : public MiniaudioObject<ma_node>
    {
    public:
        ma_node* GetMiniaudioObject()
        {
            return reinterpret_cast<ma_node*>(&_Proxy);
        }

        NodeBase(
            ma_node_graph* graph,
            size_t inputBusCount = 0,
            size_t outputBusCount = 0,
            ma_uint32 flags = 0
        )
            : _VTable {
                NodeBase::ProcessCallback,
                NodeBase::GetRequiredInputFrameCountCallback,
                0, // inputBusCount
                0, // outputBusCount
                0  // flags
            }
        {
            _Proxy.thisNode = this;
        }

        ma_result Init(
            ma_node_graph* graph,
            ma_uint8 inputBusCount = 0,
            ma_uint8 outputBusCount = 0,
            ma_uint32 flags = 0
        )
        {
            _VTable.inputBusCount = inputBusCount;
            _VTable.outputBusCount = outputBusCount;
            _VTable.flags = flags;

            ma_node_config config = ma_node_config_init();
            config.vtable = &_VTable;
            config.inputBusCount = inputBusCount;
            config.outputBusCount = outputBusCount;

            return ma_node_init(graph, &config, nullptr, GetMiniaudioObject());
        }

        virtual ~NodeBase()
        {
            ma_node_uninit(GetMiniaudioObject(), nullptr);
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
            if (NodeBase* nodeBase = CastToThis(node)) {
                nodeBase->Process(inputBuffers, inputFrameCount, outputBuffers, outputFrameCount);
            }
        }

        static ma_result GetRequiredInputFrameCountCallback(ma_node* node, ma_uint32 outputFrameCount, ma_uint32* inputFrameCount)
        {
            if (NodeBase* nodeBase = CastToThis(node)) {
                return nodeBase->GetRequiredInputFrameCount(outputFrameCount, inputFrameCount);
            }

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

    // TODO: make a standard type wrapper, then a CustomBase derived class
    class DataSourceBase : public MiniaudioObject<ma_data_source>
    {
    public:
        ma_data_source* GetMiniaudioObject()
        {
            return reinterpret_cast<ma_data_source*>(&_Proxy);
        }

        DataSourceBase(ma_uint32 flags = 0)
            : _VTable {
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
            return ma_data_source_init(&config, GetMiniaudioObject());
        }

        virtual ~DataSourceBase()
        {
            ma_data_source_uninit(GetMiniaudioObject());
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
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->Read(buffer, frameCount, framesRead);
            }

            return MA_INVALID_DATA;
        }

        static ma_result StaticSeek(ma_data_source* dataSource, ma_uint64 frameIndex)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->Seek(frameIndex);
            }

            return MA_INVALID_DATA;
        }

        static ma_result StaticGetDataFormat(ma_data_source* dataSource, ma_format* format, ma_uint32* channelCount, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCap)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->GetDataFormat(format, channelCount, sampleRate, channelMap, channelMapCap);
            }

            return MA_INVALID_DATA;
        }

        static ma_result StaticGetCursor(ma_data_source* dataSource, ma_uint64* cursor)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->GetCursor(cursor);
            }

            *cursor = 0;

            return MA_INVALID_DATA;
        }

        static ma_result StaticGetLength(ma_data_source* dataSource, ma_uint64* length)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->GetLength(length);
            }

            *length = 0;

            return MA_INVALID_DATA;
        }

        static ma_result StaticSetLooping(ma_data_source* dataSource, ma_bool32 isLooping)
        {
            if (DataSourceBase* dataSourceBase = CastToThis(dataSource)) {
                return dataSourceBase->SetLooping(isLooping);
            }

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

    // TODO: check if API wrapper is complete
    class DataSourceNode : public MiniaudioObject<ma_data_source_node>
    {
    public:
        // TODO: complete configuration parameters access
        ma_result Init(
            ma_data_source* dataSource,
            ma_node_graph* graph,
            ma_allocation_callbacks* allocationCallbacks = nullptr
        )
        {
            ma_data_source_node_config config = ma_data_source_node_config_init(dataSource);
            return ma_data_source_node_init(graph, &config, allocationCallbacks, &_DataSourceNode);
        }

        ~DataSourceNode()
        {
            ma_data_source_node_uninit(&_DataSourceNode, nullptr);
        }

        ma_data_source_node* GetMiniaudioObject()
        {
            return &_DataSourceNode;
        }

    private:
        ma_data_source_node _DataSourceNode;
    };

    // TODO: check if API wrapper is complete
    class Graph : public MiniaudioObject<ma_node_graph>
    {
    public:
        ma_node_graph* GetMiniaudioObject()
        {
            return &_Graph;
        }

        ma_result Init(
            ma_uint32 channelCount = 1,
            ma_uint16 nodeCacheCapInFrames = MA_DEFAULT_NODE_CACHE_CAP_IN_FRAMES_PER_BUS,
            const ma_allocation_callbacks* allocationCallbacks = nullptr
        )
        {
            ma_node_graph_config config = ma_node_graph_config_init(channelCount);
            config.nodeCacheCapInFrames = nodeCacheCapInFrames;
            return ma_node_graph_init(&config, allocationCallbacks, &_Graph);
        }

        ma_result Read(void* renderBuffer, size_t frameCount, size_t* framesRead)
        {
            return ma_node_graph_read_pcm_frames(&_Graph, renderBuffer, frameCount, framesRead);
        }

        ma_result DetachNodeFull(ma_node* node)
        {
            return ma_node_detach_full(node);
        }

        ma_result AttachToEndpoint(ma_node* node)
        {
            return ma_node_attach_output_bus(node, 0,  ma_node_graph_get_endpoint(&_Graph), 0);
        }

    private:
        ma_node_graph _Graph;
    };

    // TODO: check if API wrapper is complete
    class CustomDeviceBase : public MiniaudioObject<ma_device>
    {
    public:
        ma_device* GetMiniaudioObject()
        {
            return &_Device;
        }

        const ma_device_config* GetConfig()
        {
            return &_Config;
        }

        ma_result Init(
            ma_device_config config,
            ma_context* context = nullptr
        )
        {
            _Config = config;
            _Config.dataCallback = CustomDeviceBase::StaticDataCallback;
            _Config.pUserData = this;

            return ma_device_init(context, &_Config, &_Device);
        }

        virtual ~CustomDeviceBase()
        {
            ma_device_uninit(&_Device);
        }

        ma_result Start()
        {
            return ma_device_start(&_Device);
        }

        ma_result Stop()
        {
            return ma_device_stop(&_Device);
        }

        ma_result InitEx(const ma_backend backends[], ma_uint32 backendCount, const ma_context_config* contextConfig, const ma_device_config* deviceConfig)
        {
            _Config = *deviceConfig;
            return ma_device_init_ex(backends, backendCount, contextConfig, &_Config, &_Device);
        }

        ma_context* GetContext()
        {
            return ma_device_get_context(&_Device);
        }

        ma_log* GetLog()
        {
            return ma_device_get_log(&_Device);
        }

        ma_result GetInfo(ma_device_type type, ma_device_info* deviceInfo)
        {
            return ma_device_get_info(&_Device, type, deviceInfo);
        }

        ma_result GetName(ma_device_type type, char* pName, size_t nameCap, size_t* lengthNotIncludingNullTerminator)
        {
            return ma_device_get_name(&_Device, type, pName, nameCap, lengthNotIncludingNullTerminator);
        }

        ma_bool32 IsStarted() const
        {
            return ma_device_is_started(&_Device);
        }

        ma_device_state GetState() const
        {
            return ma_device_get_state(&_Device);
        }

        ma_result PostInit(ma_device_type deviceType, const ma_device_descriptor* playbackDescriptor, const ma_device_descriptor* captureDescriptor)
        {
            return ma_device_post_init(&_Device, deviceType, playbackDescriptor, captureDescriptor);
        }

        ma_result SetMasterVolume(float volume)
        {
            return ma_device_set_master_volume(&_Device, volume);
        }

        ma_result GetMasterVolume(float* volume)
        {
            return ma_device_get_master_volume(&_Device, volume);
        }

        ma_result SetMasterVolumeDb(float gainDB)
        {
            return ma_device_set_master_volume_db(&_Device, gainDB);
        }

        ma_result GetMasterVolumeDb(float* gainDB)
        {
            return ma_device_get_master_volume_db(&_Device, gainDB);
        }

        ma_result HandleBackendDataCallback(void* output, const void* input, ma_uint32 frameCount)
        {
            return ma_device_handle_backend_data_callback(&_Device, output, input, frameCount);
        }

        static ma_uint32 CalculateBufferSizeInFramesFromDescriptor(const ma_device_descriptor* descriptor, ma_uint32 nativeSampleRate, ma_performance_profile performanceProfile)
        {
            return ma_calculate_buffer_size_in_frames_from_descriptor(descriptor, nativeSampleRate, performanceProfile);
        }

        virtual void DataCallback(void* renderBuffer, const void* captureBuffer, ma_uint32 frameCount) = 0;

    private:
        static CustomDeviceBase* CastToThis(ma_device* device)
        {
            return reinterpret_cast<CustomDeviceBase*>(device->pUserData);
        }

        static void StaticDataCallback(ma_device* device, void* renderBuffer, const void* captureBuffer, ma_uint32 frameCount)
        {
            if (CustomDeviceBase* deviceBase = CastToThis(device)) {
                deviceBase->DataCallback(renderBuffer, captureBuffer, frameCount);
            }
        }

    private:
        ma_device_config _Config;
        ma_device _Device;
    };

    class Context : public MiniaudioObject<ma_context>
    {
    public:
        ma_context* GetMiniaudioObject()
        {
            return &_Context;
        }

        const ma_context_config* GetConfig()
        {
            return &_Config;
        }

        ma_result Init(
            const ma_backend backends[] = nullptr,
            ma_uint32 backendCount = 0,
            ma_context_config config = ma_context_config_init()
        )
        {
            _Config = config;
            return ma_context_init(backends, backendCount, GetConfig(), GetMiniaudioObject());
        }

        ~Context()
        {
            ma_context_uninit(GetMiniaudioObject());
        }

        ma_result EnumerateDevices(ma_enum_devices_callback_proc callback, void* userData)
        {
            return ma_context_enumerate_devices(GetMiniaudioObject(), callback, userData);
        }

        ma_result GetDevices(ma_device_info** playbackDeviceInfos, ma_uint32* playbackDeviceCount, ma_device_info** captureDeviceInfos, ma_uint32* captureDeviceCount)
        {
            return ma_context_get_devices(GetMiniaudioObject(), playbackDeviceInfos, playbackDeviceCount, captureDeviceInfos, captureDeviceCount);
        }

        ma_result GetDeviceInfo(ma_device_type deviceType, const ma_device_id* deviceID, ma_device_info* deviceInfo)
        {
            return ma_context_get_device_info(GetMiniaudioObject(), deviceType, deviceID, deviceInfo);
        }

    private:
        ma_context _Context;
        ma_context_config _Config;
    };

    class Engine : public MiniaudioObject<ma_engine>
    {
    public:
        ma_engine* GetMiniaudioObject()
        {
            return &_Engine;
        }

        ~Engine()
        {
            ma_engine_uninit(GetMiniaudioObject());
        }

        ma_result Init(const ma_engine_config& config = ma_engine_config_init())
        {
            return ma_engine_init(&config, GetMiniaudioObject());
        }

        ma_uint64 GetTimeInPCMFrames()
        {
            return ma_engine_get_time_in_pcm_frames(GetMiniaudioObject());
        }

        ma_uint64 GetTimeInMilliseconds()
        {
            return ma_engine_get_time_in_milliseconds(GetMiniaudioObject());
        }

        ma_result ReadPCMFrames(void* framesOut, ma_uint64 frameCount, ma_uint64* framesRead)
        {
            return ma_engine_read_pcm_frames(GetMiniaudioObject(), framesOut, frameCount, framesRead);
        }

        ma_node_graph* GetNodeGraph()
        {
            return ma_engine_get_node_graph(GetMiniaudioObject());
        }

        ma_device* GetDevice()
        {
            return ma_engine_get_device(GetMiniaudioObject());
        }

        ma_log* GetLog()
        {
            return ma_engine_get_log(GetMiniaudioObject());
        }

        ma_node* GetEndpoint()
        {
            return ma_engine_get_endpoint(GetMiniaudioObject());
        }

        ma_result SetTimeInPCMFrames(ma_uint64 globalTime)
        {
            return ma_engine_set_time_in_pcm_frames(GetMiniaudioObject(), globalTime);
        }

        ma_result SetTimeInMilliseconds(ma_uint64 globalTime)
        {
            return ma_engine_set_time_in_milliseconds(GetMiniaudioObject(), globalTime);
        }

        ma_uint32 GetChannels()
        {
            return ma_engine_get_channels(GetMiniaudioObject());
        }

        ma_uint32 GetSampleRate()
        {
            return ma_engine_get_sample_rate(GetMiniaudioObject());
        }

        ma_result Start()
        {
            return ma_engine_start(GetMiniaudioObject());
        }

        ma_result Stop()
        {
            return ma_engine_stop(GetMiniaudioObject());
        }

        ma_result SetVolume(float volume)
        {
            return ma_engine_set_volume(GetMiniaudioObject(), volume);
        }

        float GetVolume()
        {
            return ma_engine_get_volume(GetMiniaudioObject());
        }

        ma_result SetGainDb(float gainDB)
        {
            return ma_engine_set_gain_db(GetMiniaudioObject(), gainDB);
        }

        float GetGainDb()
        {
            return ma_engine_get_gain_db(GetMiniaudioObject());
        }

        ma_uint32 GetListenerCount()
        {
            return ma_engine_get_listener_count(GetMiniaudioObject());
        }

        ma_uint32 FindClosestListener(float absolutePosX, float absolutePosY, float absolutePosZ)
        {
            return ma_engine_find_closest_listener(GetMiniaudioObject(), absolutePosX, absolutePosY, absolutePosZ);
        }

        void ListenerSetPosition(ma_uint32 listenerIndex, float x, float y, float z)
        {
            ma_engine_listener_set_position(GetMiniaudioObject(), listenerIndex, x, y, z);
        }

        ma_vec3f ListenerGetPosition(ma_uint32 listenerIndex)
        {
            return ma_engine_listener_get_position(GetMiniaudioObject(), listenerIndex);
        }

        void ListenerSetDirection(ma_uint32 listenerIndex, float x, float y, float z)
        {
            ma_engine_listener_set_direction(GetMiniaudioObject(), listenerIndex, x, y, z);
        }

        ma_vec3f ListenerGetDirection(ma_uint32 listenerIndex)
        {
            return ma_engine_listener_get_direction(GetMiniaudioObject(), listenerIndex);
        }

        void ListenerSetVelocity(ma_uint32 listenerIndex, float x, float y, float z)
        {
            ma_engine_listener_set_velocity(GetMiniaudioObject(), listenerIndex, x, y, z);
        }

        ma_vec3f ListenerGetVelocity(ma_uint32 listenerIndex)
        {
            return ma_engine_listener_get_velocity(GetMiniaudioObject(), listenerIndex);
        }

        void ListenerSetCone(ma_uint32 listenerIndex, float innerAngleInRadians, float outerAngleInRadians, float outerGain)
        {
            ma_engine_listener_set_cone(GetMiniaudioObject(), listenerIndex, innerAngleInRadians, outerAngleInRadians, outerGain);
        }

        void ListenerGetCone(ma_uint32 listenerIndex, float* innerAngleInRadians, float* outerAngleInRadians, float* outerGain)
        {
            ma_engine_listener_get_cone(GetMiniaudioObject(), listenerIndex, innerAngleInRadians, outerAngleInRadians, outerGain);
        }

        void ListenerSetWorldUp(ma_uint32 listenerIndex, float x, float y, float z)
        {
            ma_engine_listener_set_world_up(GetMiniaudioObject(), listenerIndex, x, y, z);
        }

        ma_vec3f ListenerGetWorldUp(ma_uint32 listenerIndex)
        {
            return ma_engine_listener_get_world_up(GetMiniaudioObject(), listenerIndex);
        }

        void ListenerSetEnabled(ma_uint32 listenerIndex, ma_bool32 isEnabled)
        {
            ma_engine_listener_set_enabled(GetMiniaudioObject(), listenerIndex, isEnabled);
        }

        ma_bool32 ListenerIsEnabled(ma_uint32 listenerIndex)
        {
            return ma_engine_listener_is_enabled(GetMiniaudioObject(), listenerIndex);
        }

#ifndef MA_NO_RESOURCE_MANAGER

        ma_resource_manager* GetResourceManager()
        {
            return ma_engine_get_resource_manager(GetMiniaudioObject());
        }

        ma_result PlaySoundEx(const char* filePath, ma_node* node, ma_uint32 nodeInputBusIndex)
        {
            return ma_engine_play_sound_ex(GetMiniaudioObject(), filePath, node, nodeInputBusIndex);
        }

        ma_result PlaySound(const char* filePath, ma_sound_group* group)
        {
            return ma_engine_play_sound(GetMiniaudioObject(), filePath, group);
        }

#endif

#if MACPP_EXTENDED_METHODS_ENABLED

public:
#ifndef MA_NO_RESOURCE_MANAGER

        ma_result InitSoundFromFile(
            ma_sound* sound,
            const char* filePath,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_from_file(GetMiniaudioObject(), filePath, flags, group, fence, sound);
        }

        ma_result InitSoundFromFile(
            ma_sound* sound,
            const wchar_t* filePath,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_from_file_w(GetMiniaudioObject(), filePath, flags, group, fence, sound);
        }

        ma_result InitSoundCopy(
            ma_sound* sound,
            const ma_sound* otherSound,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_copy(GetMiniaudioObject(), otherSound, flags, group, sound);
        }

#endif // MA_NO_RESOURCE_MANAGER

        ma_result InitSoundFromDataSource(
            ma_sound* sound,
            ma_data_source* dataSource,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr
        )
        {
            return ma_sound_init_from_data_source(GetMiniaudioObject(), dataSource, flags, group, sound);
        }

#endif

    private:
        ma_engine _Engine;
    };

    class Sound : public MiniaudioObject<ma_sound>
    {
    public:
        ma_sound* GetMiniaudioObject()
        {
            return &_Sound;
        }

        ~Sound()
        {
            ma_sound_uninit(GetMiniaudioObject());
        }

#ifndef MA_NO_RESOURCE_MANAGER

        ma_result InitFromFile(
            ma_engine* engine,
            const char* filePath,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_from_file(engine, filePath, flags, group, fence, GetMiniaudioObject());
        }

        ma_result InitFromFile(
            ma_engine* engine,
            const wchar_t* filePath,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_from_file_w(engine, filePath, flags, group, fence, GetMiniaudioObject());
        }

        ma_result InitCopy(
            ma_engine* engine,
            const ma_sound* other,
            ma_uint32 flags = 0,
            ma_sound_group* group = nullptr,
            ma_fence* fence = nullptr
        )
        {
            return ma_sound_init_copy(engine, other, flags, group, GetMiniaudioObject());
        }

#endif // MA_NO_RESOURCE_MANAGER

        ma_result InitFromDataSource(
            ma_engine* engine,
            ma_data_source* dataSource,
            ma_uint32 flags,
            ma_sound_group* group
        )
        {
            return ma_sound_init_from_data_source(engine, dataSource, flags, group, GetMiniaudioObject());
        }

        ma_result InitEx(
            ma_engine* engine,
            const ma_sound_config* config
        )
        {
            return ma_sound_init_ex(engine, config, GetMiniaudioObject());
        }

        ma_result Start()
        {
            return ma_sound_start(GetMiniaudioObject());
        }

        void SetStartTimeInPCMFrames(ma_uint64 pcmTime)
        {
            ma_sound_set_start_time_in_pcm_frames(GetMiniaudioObject(), pcmTime);
        }

        void SetStartTimeInMilliseconds(ma_uint64 millisecondsTime)
        {
            ma_sound_set_start_time_in_pcm_frames(GetMiniaudioObject(), millisecondsTime);
        }

        void SetStoptTimeInPCMFrames(ma_uint64 pcmTime)
        {
            ma_sound_set_stop_time_in_pcm_frames(GetMiniaudioObject(), pcmTime);
        }

        void SetStopTimeInMilliseconds(ma_uint64 millisecondsTime)
        {
            ma_sound_set_stop_time_in_pcm_frames(GetMiniaudioObject(), millisecondsTime);
        }

        ma_engine* GetEngine()
        {
            return ma_sound_get_engine(GetMiniaudioObject());
        }

        ma_data_source* GetDataSource()
        {
            return ma_sound_get_data_source(GetMiniaudioObject());
        }

        ma_result Stop()
        {
            return ma_sound_stop(GetMiniaudioObject());
        }

        ma_result StopWithFadeInPCMFrames(ma_uint64 fadeLengthInFrames)
        {
            return ma_sound_stop_with_fade_in_pcm_frames(GetMiniaudioObject(), fadeLengthInFrames);
        }

        ma_result StopWithFadeInMilliseconds(ma_uint64 fadeLengthInMilliseconds)
        {
            return ma_sound_stop_with_fade_in_milliseconds(GetMiniaudioObject(), fadeLengthInMilliseconds);
        }

        void SetVolume(float volume)
        {
            ma_sound_set_volume(GetMiniaudioObject(), volume);
        }

        float GetVolume()
        {
            return ma_sound_get_volume(GetMiniaudioObject());
        }

        void SetPan(float pan)
        {
            ma_sound_set_pan(GetMiniaudioObject(), pan);
        }

        float GetPan()
        {
            return ma_sound_get_pan(GetMiniaudioObject());
        }

        void SetPanMode(ma_pan_mode panMode)
        {
            ma_sound_set_pan_mode(GetMiniaudioObject(), panMode);
        }

        ma_pan_mode GetPanMode()
        {
            return ma_sound_get_pan_mode(GetMiniaudioObject());
        }

        void SetPitch(float pitch)
        {
            ma_sound_set_pitch(GetMiniaudioObject(), pitch);
        }

        float GetPitch()
        {
            return ma_sound_get_pitch(GetMiniaudioObject());
        }

        void SetSpatializationEnabled(ma_bool32 enabled)
        {
            ma_sound_set_spatialization_enabled(GetMiniaudioObject(), enabled);
        }

        ma_bool32 IsSpatializationEnabled()
        {
            return ma_sound_is_spatialization_enabled(GetMiniaudioObject());
        }

        void SetPinnedListenerIndex(ma_uint32 listenerIndex)
        {
            ma_sound_set_pinned_listener_index(GetMiniaudioObject(), listenerIndex);
        }

        ma_uint32 GetPinnedListenerIndex()
        {
            return ma_sound_get_pinned_listener_index(GetMiniaudioObject());
        }

        ma_uint32 GetListenerIndex()
        {
            return ma_sound_get_listener_index(GetMiniaudioObject());
        }

        ma_vec3f GetDirectionToListener()
        {
            return ma_sound_get_direction_to_listener(GetMiniaudioObject());
        }

        void SetPosition(float x, float y, float z)
        {
            ma_sound_set_position(GetMiniaudioObject(), x, y, z);
        }

        ma_vec3f GetPosition()
        {
            return ma_sound_get_position(GetMiniaudioObject());
        }

        void SetDirection(float x, float y, float z)
        {
            ma_sound_set_direction(GetMiniaudioObject(), x, y, z);
        }

        ma_vec3f GetDirection()
        {
            return ma_sound_get_direction(GetMiniaudioObject());
        }

        void SetVelocity(float x, float y, float z)
        {
            ma_sound_set_velocity(GetMiniaudioObject(), x, y, z);
        }

        ma_vec3f GetVelocity()
        {
            return ma_sound_get_velocity(GetMiniaudioObject());
        }

        void SetAttenuationModel(ma_attenuation_model attenuationModel)
        {
            ma_sound_set_attenuation_model(GetMiniaudioObject(), attenuationModel);
        }

        ma_attenuation_model GetAttenuationModel()
        {
            return ma_sound_get_attenuation_model(GetMiniaudioObject());
        }

        void SetPositioning(ma_positioning positioning)
        {
            ma_sound_set_positioning(GetMiniaudioObject(), positioning);
        }

        ma_positioning GetPositioning()
        {
            return ma_sound_get_positioning(GetMiniaudioObject());
        }

        void SetRolloff(float rolloff)
        {
            ma_sound_set_rolloff(GetMiniaudioObject(), rolloff);
        }

        float GetRolloff()
        {
            return ma_sound_get_rolloff(GetMiniaudioObject());
        }

        void SetMinGain(float minGain)
        {
            ma_sound_set_min_gain(GetMiniaudioObject(), minGain);
        }

        float GetMinGain()
        {
            return ma_sound_get_min_gain(GetMiniaudioObject());
        }

        void SetMaxGain(float maxGain)
        {
            ma_sound_set_max_gain(GetMiniaudioObject(), maxGain);
        }

        float GetMaxGain()
        {
            return ma_sound_get_max_gain(GetMiniaudioObject());
        }

        void SetMinDistance(float minDistance)
        {
            ma_sound_set_min_distance(GetMiniaudioObject(), minDistance);
        }

        float GetMinDistance()
        {
            return ma_sound_get_min_distance(GetMiniaudioObject());
        }

        void SetMaxDistance(float maxDistance)
        {
            ma_sound_set_max_distance(GetMiniaudioObject(), maxDistance);
        }

        float GetMaxDistance()
        {
            return ma_sound_get_max_distance(GetMiniaudioObject());
        }

        void SetCone(float innerAngleInRadians, float outerAngleInRadians, float outerGain)
        {
            ma_sound_set_cone(GetMiniaudioObject(), innerAngleInRadians, outerAngleInRadians, outerGain);
        }

        void GetCone(float* innerAngleInRadians, float* outerAngleInRadians, float* pOuterGain)
        {
            ma_sound_get_cone(GetMiniaudioObject(), innerAngleInRadians, outerAngleInRadians, pOuterGain);
        }

        void SetDopplerFactor(float dopplerFactor)
        {
            ma_sound_set_doppler_factor(GetMiniaudioObject(), dopplerFactor);
        }

        float GetDopplerFactor()
        {
            return ma_sound_get_doppler_factor(GetMiniaudioObject());
        }

        void SetDirectionalAttenuationFactor(float directionalAttenuationFactor)
        {
            ma_sound_set_directional_attenuation_factor(GetMiniaudioObject(), directionalAttenuationFactor);
        }

        float GetDirectionalAttenuationFactor()
        {
            return ma_sound_get_directional_attenuation_factor(GetMiniaudioObject());
        }

        void SetFadeInPCMFrames(float volumeBegin, float volumeEnd, ma_uint64 fadeLengthInFrames)
        {
            ma_sound_set_fade_in_pcm_frames(GetMiniaudioObject(), volumeBegin, volumeEnd, fadeLengthInFrames);
        }

        void SetFadeInMilliseconds(float volumeBegin, float volumeEnd, ma_uint64 fadeLengthInMilliseconds)
        {
            ma_sound_set_fade_in_milliseconds(GetMiniaudioObject(), volumeBegin, volumeEnd, fadeLengthInMilliseconds);
        }

        void SetFadeStartInPCMFrames(float volumeBegin, float volumeEnd, ma_uint64 fadeLengthInFrames, ma_uint64 absoluteGlobalTimeInFrames)
        {
            ma_sound_set_fade_start_in_pcm_frames(GetMiniaudioObject(), volumeBegin, volumeEnd, fadeLengthInFrames, absoluteGlobalTimeInFrames);
        }

        void SetFadeStartInMilliseconds(float volumeBegin, float volumeEnd, ma_uint64 fadeLengthInMilliseconds, ma_uint64 absoluteGlobalTimeInMilliseconds)
        {
            ma_sound_set_fade_start_in_milliseconds(GetMiniaudioObject(), volumeBegin, volumeEnd, fadeLengthInMilliseconds, absoluteGlobalTimeInMilliseconds);
        }

        float GetCurrentFadeVolume()
        {
            return ma_sound_get_current_fade_volume(GetMiniaudioObject());
        }

        void SetStopTimeWithFadeInPCMFrames(ma_uint64 stopAbsoluteGlobalTimeInFrames, ma_uint64 fadeLengthInFrames)
        {
            ma_sound_set_stop_time_with_fade_in_pcm_frames(GetMiniaudioObject(), stopAbsoluteGlobalTimeInFrames, fadeLengthInFrames);
        }

        void SetStopTimeWithFadeInMilliseconds(ma_uint64 stopAbsoluteGlobalTimeInMilliseconds, ma_uint64 fadeLengthInMilliseconds)
        {
            ma_sound_set_stop_time_with_fade_in_milliseconds(GetMiniaudioObject(), stopAbsoluteGlobalTimeInMilliseconds, fadeLengthInMilliseconds);
        }

        ma_bool32 IsPlaying()
        {
            return ma_sound_is_playing(GetMiniaudioObject());
        }

        ma_uint64 GetTimeInPCMFrames()
        {
            return ma_sound_get_time_in_pcm_frames(GetMiniaudioObject());
        }

        ma_uint64 GetTimeInMilliseconds()
        {
            return ma_sound_get_time_in_milliseconds(GetMiniaudioObject());
        }

        void SetLooping(ma_bool32 isLooping)
        {
            ma_sound_set_looping(GetMiniaudioObject(), isLooping);
        }

        ma_bool32 IsLooping()
        {
            return ma_sound_is_looping(GetMiniaudioObject());
        }

        ma_bool32 AtEnd()
        {
            return ma_sound_at_end(GetMiniaudioObject());
        }

        ma_result SeekToPCMFrame(ma_uint64 frameIndex)
        {
            return ma_sound_seek_to_pcm_frame(GetMiniaudioObject(), frameIndex);
        }

        ma_result GetDataFormat(ma_format* format, ma_uint32* channels, ma_uint32* sampleRate, ma_channel* channelMap, size_t channelMapCapacity)
        {
            return ma_sound_get_data_format(GetMiniaudioObject(), format, channels, sampleRate, channelMap, channelMapCapacity);
        }

        ma_result GetCursorInPCMFrames(ma_uint64* cursor)
        {
            return ma_sound_get_cursor_in_pcm_frames(GetMiniaudioObject(), cursor);
        }

        ma_result GetLengthInPCMFrames(ma_uint64* length)
        {
            return ma_sound_get_length_in_pcm_frames(GetMiniaudioObject(), length);
        }

        ma_result GetCursorInSeconds(float* cursor)
        {
            return ma_sound_get_cursor_in_seconds(GetMiniaudioObject(), cursor);
        }

        ma_result GetLengthInSeconds(float* length)
        {
            return ma_sound_get_length_in_seconds(GetMiniaudioObject(), length);
        }

        ma_result SetEndCallback(ma_sound_end_proc callback, void* userData)
        {
            return ma_sound_set_end_callback(GetMiniaudioObject(), callback, userData);
        }

    private:
        ma_sound _Sound;
    };

} // namespace MiniaudioCpp
