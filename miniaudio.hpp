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
#define MACPP_VERSION_REVISION 0
#define MACPP_VERSION_STRING   MA_XSTRINGIFY(MACPP_VERSION_MAJOR) "." MA_XSTRINGIFY(MACPP_VERSION_MINOR) "." MA_XSTRINGIFY(MACPP_VERSION_REVISION)

/* Extended methods refer to addition that were made on top of the `miniaudio` API. */
#define MACPP_ENABLE_EXTENDED_METHODS false

#if MACPP_ENABLE_EXTENDED_METHODS

/*

Extended methods use the following `std` classes

* std::vector
* ...

Aliases are provided for these classes for simple substitution. If you intend to use different classes here, keep in mind
that they must respect the corresponding `std` API.

*/

#include <vector>

namespace MiniaudioCpp
{
    template <class T>
    using Vector = std::vector<T>;
}

#endif // MACPP_ENABLE_EXTENDED_METHODS

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

    };

    class Decoder : public MiniaudioObject<ma_decoder>
    {
    public:
        ma_decoder* GetMiniaudioObject()
        {
            return &_Decoder;
        }

        Decoder()
        {
            memset(GetMiniaudioObject(), 0, sizeof(MiniaudioObject::Type));
        }

        /* MA_API ma_result ma_decoder_uninit(ma_decoder * pDecoder); */
        virtual ~Decoder()
        {
            ma_decoder_uninit(GetMiniaudioObject());
        }
        
        /* MA_API ma_result ma_decoder_init_file(const char* pFilePath, const ma_decoder_config* pConfig, ma_decoder* pDecoder); */
        ma_result Init(
            const char* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decoder_init_file(filePath, &config, GetMiniaudioObject());
        }

        /* MA_API ma_result ma_decoder_init_file_w(const wchar_t* pFilePath, const ma_decoder_config* pConfig, ma_decoder* pDecoder); */
        ma_result Init(
            const wchar_t* filePath,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decoder_init_file_w(filePath, &config, GetMiniaudioObject());
        }

        /* MA_API ma_result ma_decoder_init_memory(const void* pData, size_t dataSize, const ma_decoder_config* pConfig, ma_decoder* pDecoder); */
        ma_result Init(
            const void* data,
            size_t dataSize,
            ma_decoder_config config = ma_decoder_config_init_default()
        )
        {
            return ma_decoder_init_memory(data, dataSize, &config, GetMiniaudioObject());
        }

        // TODO:
        /* MA_API ma_result ma_decoder_init(ma_decoder_read_proc onRead, ma_decoder_seek_proc onSeek, void* pUserData, const ma_decoder_config* pConfig, ma_decoder* pDecoder); */
        /* MA_API ma_result ma_decoder_init_vfs(ma_vfs* pVFS, const char* pFilePath, const ma_decoder_config* pConfig, ma_decoder* pDecoder); */
        /* MA_API ma_result ma_decoder_init_vfs_w(ma_vfs * pVFS, const wchar_t* pFilePath, const ma_decoder_config * pConfig, ma_decoder * pDecoder); */
        /* MA_API ma_result ma_decoder_get_data_format(ma_decoder* pDecoder, ma_format* pFormat, ma_uint32* pChannels, ma_uint32* pSampleRate, ma_channel* pChannelMap, size_t channelMapCap); */
        /* MA_API ma_result ma_decode_from_vfs(ma_vfs* pVFS, const char* pFilePath, ma_decoder_config* pConfig, ma_uint64* pFrameCountOut, void** ppPCMFramesOut); */
        /* MA_API ma_result ma_decode_file(const char* pFilePath, ma_decoder_config* pConfig, ma_uint64* pFrameCountOut, void** ppPCMFramesOut); */
        /* MA_API ma_result ma_decode_memory(const void* pData, size_t dataSize, ma_decoder_config* pConfig, ma_uint64* pFrameCountOut, void** ppPCMFramesOut); */

        /* MA_API ma_result ma_decoder_read_pcm_frames(ma_decoder* pDecoder, void* pFramesOut, ma_uint64 frameCount, ma_uint64* pFramesRead); */
        ma_result Read(void* buffer, ma_uint64 frameCount, ma_uint64* framesRead)
        {
            return ma_decoder_read_pcm_frames(GetMiniaudioObject(), buffer, frameCount, framesRead);
        };

        /* MA_API ma_result ma_decoder_seek_to_pcm_frame(ma_decoder* pDecoder, ma_uint64 frameIndex); */
        ma_result Seek(ma_uint64 targetFrame)
        {
            return ma_decoder_seek_to_pcm_frame(GetMiniaudioObject(), targetFrame);
        }
        
        /* MA_API ma_result ma_decoder_get_available_frames(ma_decoder* pDecoder, ma_uint64* pAvailableFrames); */
        ma_result GetAvailableFrames(ma_uint64* availableFrames)
        {
            return ma_decoder_get_available_frames(GetMiniaudioObject(), availableFrames);
        }

        /* MA_API ma_result ma_decoder_get_cursor_in_pcm_frames(ma_decoder* pDecoder, ma_uint64* pCursor); */
        ma_result GetCursor(ma_uint64* cursor)
        {
            return ma_decoder_get_cursor_in_pcm_frames(GetMiniaudioObject(), cursor);
        }

        /* MA_API ma_result ma_decoder_get_length_in_pcm_frames(ma_decoder* pDecoder, ma_uint64* pLength); */
        ma_result GetLength(ma_uint64* length)
        {
            return ma_decoder_get_length_in_pcm_frames(GetMiniaudioObject(), length);
        }

#if MACPP_ENABLE_EXTENDED_METHODS
        ma_result Decode(Vector<float>& samples, size_t fromFrame = 0, size_t frameCount = 0)
        {
            if (fromFrame > 0)
                MACPP_CHECK_RESULT(Seek(fromFrame));

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
#endif // MACPP_ENABLE_EXTENDED_METHODS

    private:
        ma_decoder _Decoder;
    };

    // TODO: check if API wrapper is complete
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

    // TODO: check if API wrapper is complete
    class DataSourceBase : public MiniaudioObject<ma_data_source>
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
            return ma_data_source_init(&config, GetMiniaudioObject());
        }

        virtual ~DataSourceBase()
        {
            ma_data_source_uninit(GetMiniaudioObject());
        }

        ma_data_source* GetMiniaudioObject()
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

    // TODO: check if API wrapper is complete
    class DataSourceNode : public MiniaudioObject<ma_data_source_node>
    {
    public:
        DataSourceNode()
        {
        }

        // TODO: complete configuration parameters access
        ma_result Init(ma_data_source* dataSource, ma_node_graph* graph, ma_allocation_callbacks* allocationCallbacks = nullptr)
        {
            ma_data_source_node_config config = ma_data_source_node_config_init(dataSource);
            return ma_data_source_node_init(graph, &config, allocationCallbacks, GetMiniaudioObject());
        }

        ~DataSourceNode()
        {
            ma_data_source_node_uninit(GetMiniaudioObject(), nullptr);
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
        Graph()
            : _Graph()
        {
        }

        ma_result Init(
            ma_uint32 channelCount = 1,
            ma_uint16 nodeCacheCapInFrames = MA_DEFAULT_NODE_CACHE_CAP_IN_FRAMES_PER_BUS,
            const ma_allocation_callbacks* allocationCallbacks = nullptr
        )
        {
            ma_node_graph_config config = ma_node_graph_config_init(channelCount);
            config.nodeCacheCapInFrames = nodeCacheCapInFrames;
            return ma_node_graph_init(&config, allocationCallbacks, GetMiniaudioObject());
        }

        ma_result Read(void* renderBuffer, size_t frameCount, size_t* framesRead)
        {
            return ma_node_graph_read_pcm_frames(GetMiniaudioObject(), renderBuffer, frameCount, framesRead);
        }

        ma_result DetachNodeFull(ma_node* node)
        {
            return ma_node_detach_full(node);
        }

        ma_result AttachToEndpoint(ma_node* node)
        {
            return ma_node_attach_output_bus(node, 0,  ma_node_graph_get_endpoint(GetMiniaudioObject()), 0);
        }

        ma_node_graph* GetMiniaudioObject()
        {
            return &_Graph;
        }

    private:
        ma_node_graph _Graph;
    };

    // TODO: check if API wrapper is complete
    class DeviceBase : public MiniaudioObject<ma_device>
    {
    public:
        ma_result Init(
            ma_device_type deviceType = ma_device_type_playback,
            ma_uint32 channelCount = 0, // Use native device channel count
            ma_uint32 sampleRate = 0, // Use native device sample rate
            ma_format format = ma_format_f32,
            ma_context* context = nullptr
        )
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

            return ma_device_init(context, &config, GetMiniaudioObject());
        }

        virtual ~DeviceBase()
        {
            ma_device_uninit(GetMiniaudioObject());
        }

        ma_result Start()
        {
            return ma_device_start(GetMiniaudioObject());
        }

        ma_result Stop()
        {
            return ma_device_stop(GetMiniaudioObject());
        }

        ma_device* GetMiniaudioObject()
        {
            return &_Device;
        }

        virtual void DataCallback(void* renderBuffer, const void* captureBuffer, ma_uint32 frameCount) = 0;

#if MACPP_ENABLE_EXTENDED_METHODS
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
#endif // MACPP_ENABLE_EXTENDED_METHODS

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

    private:
        ma_device _Device;
    };

    class Context : public MiniaudioObject<ma_context>
    {
    public:
        Context()
        {
        }

        ma_result Init(
            const ma_backend backends[] = nullptr,
            ma_uint32 backendCount = 0,
            ma_allocation_callbacks allocationBallbacks = ma_allocation_callbacks_init_default()
        )
        {
            ma_context_config config = ma_context_config_init();
            config.allocationCallbacks = allocationBallbacks;
            return ma_context_init(backends, backendCount, &config, GetMiniaudioObject());
        }

        ma_context* GetMiniaudioObject()
        {
            return &_Context;
        }

    private:
        ma_context _Context;
    };


    class Engine : public MiniaudioObject<ma_engine>
    {
    public:
        ma_engine* GetMiniaudioObject()
        {
            return &_Engine;
        }

        /* MA_API void ma_engine_uninit(ma_engine* pEngine); */
        ~Engine()
        {
            ma_engine_uninit(GetMiniaudioObject());
        }

        /* MA_API ma_result ma_engine_init(const ma_engine_config* pConfig, ma_engine* pEngine); */
        ma_result Init(const ma_engine_config config = ma_engine_config_init())
        {
            return ma_engine_init(&config, GetMiniaudioObject());
        }

        /* MA_API ma_uint64 ma_engine_get_time_in_pcm_frames(const ma_engine* pEngine); */
        ma_uint64 GetTimeInPCMFrames()
        {
            return ma_engine_get_time_in_pcm_frames(GetMiniaudioObject());
        }

        /* MA_API ma_uint64 ma_engine_get_time_in_milliseconds(const ma_engine* pEngine); */
        ma_uint64 GetTimeInMilliseconds()
        {
            return ma_engine_get_time_in_milliseconds(GetMiniaudioObject());
        }

        /* MA_API ma_result ma_engine_read_pcm_frames(ma_engine* pEngine, void* pFramesOut, ma_uint64 frameCount, ma_uint64* pFramesRead); */
        /* MA_API ma_node_graph* ma_engine_get_node_graph(ma_engine* pEngine); */
        /* MA_API ma_device* ma_engine_get_device(ma_engine* pEngine); */
        /* MA_API ma_log* ma_engine_get_log(ma_engine* pEngine); */
        /* MA_API ma_node* ma_engine_get_endpoint(ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_set_time_in_pcm_frames(ma_engine* pEngine, ma_uint64 globalTime); */
        /* MA_API ma_result ma_engine_set_time_in_milliseconds(ma_engine* pEngine, ma_uint64 globalTime); */
        /* MA_API ma_uint64 ma_engine_get_time(const ma_engine* pEngine);                  /* Deprecated. Use ma_engine_get_time_in_pcm_frames(). Will be removed in version 0.12. */
        /* MA_API ma_result ma_engine_set_time(ma_engine* pEngine, ma_uint64 globalTime);  /* Deprecated. Use ma_engine_set_time_in_pcm_frames(). Will be removed in version 0.12. */
        /* MA_API ma_uint32 ma_engine_get_channels(const ma_engine* pEngine); */
        /* MA_API ma_uint32 ma_engine_get_sample_rate(const ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_start(ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_stop(ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_set_volume(ma_engine* pEngine, float volume); */
        /* MA_API float ma_engine_get_volume(ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_set_gain_db(ma_engine* pEngine, float gainDB); */
        /* MA_API float ma_engine_get_gain_db(ma_engine* pEngine); */
        /* MA_API ma_uint32 ma_engine_get_listener_count(const ma_engine* pEngine); */
        /* MA_API ma_uint32 ma_engine_find_closest_listener(const ma_engine* pEngine, float absolutePosX, float absolutePosY, float absolutePosZ); */
        /* MA_API void ma_engine_listener_set_position(ma_engine* pEngine, ma_uint32 listenerIndex, float x, float y, float z); */
        /* MA_API ma_vec3f ma_engine_listener_get_position(const ma_engine* pEngine, ma_uint32 listenerIndex); */
        /* MA_API void ma_engine_listener_set_direction(ma_engine* pEngine, ma_uint32 listenerIndex, float x, float y, float z); */
        /* MA_API ma_vec3f ma_engine_listener_get_direction(const ma_engine* pEngine, ma_uint32 listenerIndex); */
        /* MA_API void ma_engine_listener_set_velocity(ma_engine* pEngine, ma_uint32 listenerIndex, float x, float y, float z); */
        /* MA_API ma_vec3f ma_engine_listener_get_velocity(const ma_engine* pEngine, ma_uint32 listenerIndex); */
        /* MA_API void ma_engine_listener_set_cone(ma_engine* pEngine, ma_uint32 listenerIndex, float innerAngleInRadians, float outerAngleInRadians, float outerGain); */
        /* MA_API void ma_engine_listener_get_cone(const ma_engine* pEngine, ma_uint32 listenerIndex, float* pInnerAngleInRadians, float* pOuterAngleInRadians, float* pOuterGain); */
        /* MA_API void ma_engine_listener_set_world_up(ma_engine* pEngine, ma_uint32 listenerIndex, float x, float y, float z); */
        /* MA_API ma_vec3f ma_engine_listener_get_world_up(const ma_engine* pEngine, ma_uint32 listenerIndex); */
        /* MA_API void ma_engine_listener_set_enabled(ma_engine* pEngine, ma_uint32 listenerIndex, ma_bool32 isEnabled); */
        /* MA_API ma_bool32 ma_engine_listener_is_enabled(const ma_engine* pEngine, ma_uint32 listenerIndex); */

#ifndef MA_NO_RESOURCE_MANAGER

        /* MA_API ma_resource_manager* ma_engine_get_resource_manager(ma_engine* pEngine); */
        /* MA_API ma_result ma_engine_play_sound_ex(ma_engine* pEngine, const char* pFilePath, ma_node* pNode, ma_uint32 nodeInputBusIndex); */
        /* MA_API ma_result ma_engine_play_sound(ma_engine* pEngine, const char* pFilePath, ma_sound_group* pGroup); */

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

        /* MA_API void ma_sound_uninit(ma_sound* pSound);*/
        ~Sound()
        {
            ma_sound_uninit(GetMiniaudioObject());
        }

#ifndef MA_NO_RESOURCE_MANAGER

        /* MA_API ma_result ma_sound_init_from_file(ma_engine* pEngine, const char* pFilePath, ma_uint32 flags, ma_sound_group* pGroup, ma_fence* pDoneFence, ma_sound* pSound); */
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

        /* MA_API ma_result ma_sound_init_from_file_w(ma_engine* pEngine, const wchar_t* pFilePath, ma_uint32 flags, ma_sound_group* pGroup, ma_fence* pDoneFence, ma_sound* pSound); */
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

        /* MA_API ma_result ma_sound_init_copy(ma_engine* pEngine, const ma_sound* pExistingSound, ma_uint32 flags, ma_sound_group* pGroup, ma_sound* pSound); */
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

        /* MA_API ma_result ma_sound_start(ma_sound* pSound); */
        ma_result Start()
        {
            return ma_sound_start(GetMiniaudioObject());
        }

        /* MA_API void ma_sound_set_start_time_in_pcm_frames(ma_sound* pSound, ma_uint64 absoluteGlobalTimeInFrames); */
        void SetStartTimeInPCMFrames(ma_uint64 pcmTime)
        {
            ma_sound_set_start_time_in_pcm_frames(GetMiniaudioObject(), pcmTime);
        }

        /* MA_API void ma_sound_set_start_time_in_milliseconds(ma_sound* pSound, ma_uint64 absoluteGlobalTimeInMilliseconds); */
        void SetStartTimeInMilliseconds(ma_uint64 millisecondsTime)
        {
            ma_sound_set_start_time_in_pcm_frames(GetMiniaudioObject(), millisecondsTime);
        }

        /* MA_API void ma_sound_set_stop_time_in_pcm_frames(ma_sound* pSound, ma_uint64 absoluteGlobalTimeInFrames); */
        void SetStoptTimeInPCMFrames(ma_uint64 pcmTime)
        {
            ma_sound_set_stop_time_in_pcm_frames(GetMiniaudioObject(), pcmTime);
        }

        /* MA_API void ma_sound_set_stop_time_in_milliseconds(ma_sound* pSound, ma_uint64 absoluteGlobalTimeInMilliseconds); */
        void SetStopTimeInMilliseconds(ma_uint64 millisecondsTime)
        {
            ma_sound_set_stop_time_in_pcm_frames(GetMiniaudioObject(), millisecondsTime);
        }

        // TODO:
        /* MA_API ma_result ma_sound_init_from_data_source(ma_engine* pEngine, ma_data_source* pDataSource, ma_uint32 flags, ma_sound_group* pGroup, ma_sound* pSound); */
        /* MA_API ma_result ma_sound_init_ex(ma_engine* pEngine, const ma_sound_config* pConfig, ma_sound* pSound); */
        /* MA_API ma_engine* ma_sound_get_engine(const ma_sound* pSound); */
        /* MA_API ma_data_source* ma_sound_get_data_source(const ma_sound* pSound); */
        /* MA_API ma_result ma_sound_stop(ma_sound* pSound); */
        /* MA_API ma_result ma_sound_stop_with_fade_in_pcm_frames(ma_sound* pSound, ma_uint64 fadeLengthInFrames); */
        /* MA_API ma_result ma_sound_stop_with_fade_in_milliseconds(ma_sound* pSound, ma_uint64 fadeLengthInFrames); */
        /* MA_API void ma_sound_set_volume(ma_sound* pSound, float volume); */
        /* MA_API float ma_sound_get_volume(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_pan(ma_sound* pSound, float pan); */
        /* MA_API float ma_sound_get_pan(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_pan_mode(ma_sound* pSound, ma_pan_mode panMode); */
        /* MA_API ma_pan_mode ma_sound_get_pan_mode(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_pitch(ma_sound* pSound, float pitch); */
        /* MA_API float ma_sound_get_pitch(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_spatialization_enabled(ma_sound* pSound, ma_bool32 enabled); */
        /* MA_API ma_bool32 ma_sound_is_spatialization_enabled(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_pinned_listener_index(ma_sound* pSound, ma_uint32 listenerIndex); */
        /* MA_API ma_uint32 ma_sound_get_pinned_listener_index(const ma_sound* pSound); */
        /* MA_API ma_uint32 ma_sound_get_listener_index(const ma_sound* pSound); */
        /* MA_API ma_vec3f ma_sound_get_direction_to_listener(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_position(ma_sound* pSound, float x, float y, float z); */
        /* MA_API ma_vec3f ma_sound_get_position(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_direction(ma_sound* pSound, float x, float y, float z); */
        /* MA_API ma_vec3f ma_sound_get_direction(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_velocity(ma_sound* pSound, float x, float y, float z); */
        /* MA_API ma_vec3f ma_sound_get_velocity(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_attenuation_model(ma_sound* pSound, ma_attenuation_model attenuationModel); */
        /* MA_API ma_attenuation_model ma_sound_get_attenuation_model(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_positioning(ma_sound* pSound, ma_positioning positioning); */
        /* MA_API ma_positioning ma_sound_get_positioning(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_rolloff(ma_sound* pSound, float rolloff); */
        /* MA_API float ma_sound_get_rolloff(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_min_gain(ma_sound* pSound, float minGain); */
        /* MA_API float ma_sound_get_min_gain(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_max_gain(ma_sound* pSound, float maxGain); */
        /* MA_API float ma_sound_get_max_gain(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_min_distance(ma_sound* pSound, float minDistance); */
        /* MA_API float ma_sound_get_min_distance(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_max_distance(ma_sound* pSound, float maxDistance); */
        /* MA_API float ma_sound_get_max_distance(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_cone(ma_sound* pSound, float innerAngleInRadians, float outerAngleInRadians, float outerGain); */
        /* MA_API void ma_sound_get_cone(const ma_sound* pSound, float* pInnerAngleInRadians, float* pOuterAngleInRadians, float* pOuterGain); */
        /* MA_API void ma_sound_set_doppler_factor(ma_sound* pSound, float dopplerFactor); */
        /* MA_API float ma_sound_get_doppler_factor(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_directional_attenuation_factor(ma_sound* pSound, float directionalAttenuationFactor); */
        /* MA_API float ma_sound_get_directional_attenuation_factor(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_fade_in_pcm_frames(ma_sound* pSound, float volumeBeg, float volumeEnd, ma_uint64 fadeLengthInFrames); */
        /* MA_API void ma_sound_set_fade_in_milliseconds(ma_sound* pSound, float volumeBeg, float volumeEnd, ma_uint64 fadeLengthInMilliseconds); */
        /* MA_API void ma_sound_set_fade_start_in_pcm_frames(ma_sound* pSound, float volumeBeg, float volumeEnd, ma_uint64 fadeLengthInFrames, ma_uint64 absoluteGlobalTimeInFrames); */
        /* MA_API void ma_sound_set_fade_start_in_milliseconds(ma_sound* pSound, float volumeBeg, float volumeEnd, ma_uint64 fadeLengthInMilliseconds, ma_uint64 absoluteGlobalTimeInMilliseconds); */
        /* MA_API float ma_sound_get_current_fade_volume(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_stop_time_with_fade_in_pcm_frames(ma_sound* pSound, ma_uint64 stopAbsoluteGlobalTimeInFrames, ma_uint64 fadeLengthInFrames); */
        /* MA_API void ma_sound_set_stop_time_with_fade_in_milliseconds(ma_sound* pSound, ma_uint64 stopAbsoluteGlobalTimeInMilliseconds, ma_uint64 fadeLengthInMilliseconds); */
        /* MA_API ma_bool32 ma_sound_is_playing(const ma_sound* pSound); */
        /* MA_API ma_uint64 ma_sound_get_time_in_pcm_frames(const ma_sound* pSound); */
        /* MA_API ma_uint64 ma_sound_get_time_in_milliseconds(const ma_sound* pSound); */
        /* MA_API void ma_sound_set_looping(ma_sound* pSound, ma_bool32 isLooping); */
        /* MA_API ma_bool32 ma_sound_is_looping(const ma_sound* pSound); */
        /* MA_API ma_bool32 ma_sound_at_end(const ma_sound* pSound); */
        /* MA_API ma_result ma_sound_seek_to_pcm_frame(ma_sound* pSound, ma_uint64 frameIndex); /* Just a wrapper around ma_data_source_seek_to_pcm_frame(). */ 
        /* MA_API ma_result ma_sound_get_data_format(ma_sound* pSound, ma_format* pFormat, ma_uint32* pChannels, ma_uint32* pSampleRate, ma_channel* pChannelMap, size_t channelMapCap); */
        /* MA_API ma_result ma_sound_get_cursor_in_pcm_frames(ma_sound* pSound, ma_uint64* pCursor); */
        /* MA_API ma_result ma_sound_get_length_in_pcm_frames(ma_sound* pSound, ma_uint64* pLength); */
        /* MA_API ma_result ma_sound_get_cursor_in_seconds(ma_sound* pSound, float* pCursor); */
        /* MA_API ma_result ma_sound_get_length_in_seconds(ma_sound* pSound, float* pLength); */
        /* MA_API ma_result ma_sound_set_end_callback(ma_sound* pSound, ma_sound_end_proc callback, void* pUserData); */

    private:
        ma_sound _Sound;
    };

#if 0
namespace Extended
{
    class PCMData
    {
    public:
        PCMData()
            : _Samples()
            , _Format()
        {
        }

        Vector<float>& GetSamplesForWriting()
        {
            return _Samples;
        }

        const Vector<float>& GetSamples() const
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
        Vector<float> _Samples;
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
            MACPP_CHECK_RESULT(result);
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

            const Vector<float>& samples = _CurrentData->GetSamples();
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

}// namespace MiniaudioCpp::Extended

#endif // #if 0

} // namespace MiniaudioCpp
