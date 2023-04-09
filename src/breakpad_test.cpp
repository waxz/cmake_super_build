//
// Created by waxz on 4/8/23.
//
#include <client/linux/handler/exception_handler.h>
#include <common/linux/http_upload.h>
//#include <common/linux/google_crashdump_uploader.h>
#include "breakpad_func.h"

static bool dumpCallback(const google_breakpad::MinidumpDescriptor &descriptor,
                         void *context, bool succeeded) {
    printf("Dump path: %s\n", descriptor.path());
    (void) context;
    if (succeeded) {
        std::map<string, string> parameters;
        std::map<string, string> files;
        std::string proxy_host;
        std::string proxy_userpasswd;
        std::string url(
                "http://127.0.0.1:44444");

        // Add any attributes to the parameters map.
        // Note that several attributes are automatically extracted.
        parameters["product_name"] = "foo";
        parameters["version"] = "0.1.0";
//        parameters["file"] = descriptor.path();

        files["upload_file_minidump"] = descriptor.path();

        std::string response, error;
        bool success = google_breakpad::HTTPUpload::SendRequest(url,
                                                                parameters,
                                                                files,
                                                                proxy_host,
                                                                proxy_userpasswd,
                                                                "",
                                                                &response,
                                                                NULL,
                                                                &error);

    }
    return succeeded;
}


int main(int argc, char *argv[])
{
    google_breakpad::MinidumpDescriptor descriptor("./");
    google_breakpad::ExceptionHandler eh(descriptor, nullptr, dumpCallback, nullptr, true, -1);
    crash();
    return 0;
}