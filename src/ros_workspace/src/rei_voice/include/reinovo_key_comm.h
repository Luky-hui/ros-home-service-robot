#ifndef REINOVO_KEY_COMM_H
#define REINOVO_KEY_COMM_H

#include <string>

#if defined(_WIN32)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct Response {
    int code;
    std::string msg;
};

struct LoginResponse {
    int code;
    std::string msg;
    std::string token;
};

struct CallResponse {
    int code;
    std::string msg;
    int used;
    int remain;
};

struct CheckTokenResponse {
    int code;
    std::string msg;
    std::string email;
    int role;
};

API_EXPORT bool reinovo_init();
API_EXPORT Response send_email_code(const char* email);
API_EXPORT CheckTokenResponse check_token();
API_EXPORT Response user_register(const char* email, const char* password, const char* code);
API_EXPORT LoginResponse user_login(const char* email, const char* pwd, const char* code);
API_EXPORT Response reset_pwd(const char* email, const char* password, const char* code);
API_EXPORT CallResponse call_api();

#ifdef __cplusplus
}
#endif

#endif