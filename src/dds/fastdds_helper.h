//
// Created by waxz on 9/11/23.
//

#ifndef CMAKE_SUPER_BUILD_FASTDDS_HELPER_H
#define CMAKE_SUPER_BUILD_FASTDDS_HELPER_H



#ifdef __cplusplus
extern "C" {
#endif

    /*

     toml file define some TOPIC:
     each TOPIC define:
         - write_read
         - topic_name
         - topic_type
         - read_function, for reader, based on topic_type
         - write_function, for writer, based on topic_type
         - process_function, for reader,  base on tcc source
         - prepare_function, for writer,  base on tcc source

     */


    /*

     */





    typedef struct LaserScan{


    } LaserScan,* LaserScanPtr;


    /// create all dds TOPIC from toml file
    /// \param filename: toml filename
    /// \return HANDLER pointer
    void* dds_create_from_toml(const char* filename);

    /// free dds HANDLER
    /// \param handler: HANDLER pointer
    /// \return
    void dds_clean_handler(void* vp_handler);


    /// get symbol pointer like variable or function from TOPIC's tcc
    /// \param handler
    /// \param channel_name
    /// \param symbol
    /// \return
    void* dds_get_symbol(void* vp_handler, const char* channel_name, const char* symbol);

    /// trigger TOPIC's write_function
    /// \param handler
    /// \param channel_name
    /// \return
    int dds_write_channel(void* vp_handler, const char* channel_name);


    /// trigger TOPIC's read_function
    /// \param handler
    /// \param channel_name
    /// \return
    int dds_read_channel(void* vp_handler, const char* channel_name);


#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_FASTDDS_HELPER_H
