#include <cstdlib>
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsrc.h>
#include <glib-unix.h>
#include <dlfcn.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

using namespace std;

#define USE(x) ((void)(x))

static GstPipeline *gst_pipeline = nullptr;
static string launch_string;
static GstElement *appsrc_;

GstClockTime timestamp = 0;
GstClockTime usec = 1000000;
static int w = 2896;
static int h = 2896;
static void *ptr = nullptr;

static gboolean feed_function(gpointer user_data) {
    GstBuffer *buffer;
    guint size;
    GstFlowReturn ret;
    GstMapInfo map = {0};

    size = (w*h*3)>>1;
    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    buffer->pts = timestamp;

    gst_buffer_map (buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, ptr , size);
    gst_buffer_unmap(buffer, &map);

    g_signal_emit_by_name (appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    timestamp += (usec*1000); // nano second
    return G_SOURCE_CONTINUE;
}

int main(int argc, char** argv) {
    USE(argc);
    USE(argv);

    gst_init (&argc, &argv);

    GMainLoop *main_loop;
    main_loop = g_main_loop_new (NULL, FALSE);
    ostringstream launch_stream;

    launch_stream
    << "appsrc name=mysource ! "
    << "video/x-raw,width="<< w <<",height="<< h <<",framerate=1/1,format=I420 ! "
    << "omxh264enc ! h264parse ! qtmux ! "
    << "filesink location=a.mp4 ";

    launch_string = launch_stream.str();

    g_print("Using launch string: %s\n", launch_string.c_str());

    GError *error = nullptr;
    gst_pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

    if (gst_pipeline == nullptr) {
        g_print( "Failed to parse launch: %s\n", error->message);
        return -1;
    }
    if(error) g_error_free(error);

    appsrc_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), "mysource");
    gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);

    guint size;
    size = (w*h*3)>>1;
    FILE *fp = fopen ("/home/nvidia/a.yuv", "rb");
    ptr = malloc(size);
    fread(ptr, size, 1, fp);
    fclose(fp);

    gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING); 

    for (int i=0; i<30; i++) {
        feed_function(nullptr);
        usleep(usec);
    }
    gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
    usleep(500000); // wait for qtmux to complete muxing

    gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(gst_pipeline));
    g_main_loop_unref(main_loop);

    free(ptr);
    g_print("going to exit \n");
    return 0;
}
