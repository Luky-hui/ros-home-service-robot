#include "reinovo_key_comm.h"
#include <iostream>
#include <limits>

using namespace std;

void clear_cin() {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

void show_menu() {
    cout << "\n===== Reinovo Key Comm 完整版 =====" << endl;
    cout << "1. 用户注册" << endl;
    cout << "2. 用户登录" << endl;
    cout << "3. 修改密码" << endl;
    cout << "4. 接口调用（次数+1）" << endl;
    cout << "5. 检测当前token是否有效" << endl;  // <-- 新增
    cout << "0. 退出" << endl;
    cout << "请输入选项：";
}

int main() {
    if (!reinovo_init()) {
        cerr << "初始化失败！" << endl;
        return -1;
    }
    cout << "初始化成功！" << endl;

    while (true) {
        show_menu();
        int ch;
        if (!(cin >> ch)) { clear_cin(); cout << "输入错误！" << endl; continue; }
        clear_cin();

        switch (ch) {
            // ========== 注册 ==========
            case 1: {
                string email;
                cout << "\n请输入注册邮箱：";
                cin >> email;

                cout << "1 发送验证码" << endl;
                cout << "0 退出" << endl;
                cout << "请选择：";
                int opt; cin >> opt; clear_cin();
                if (opt != 1) break;

                auto ret = send_email_code(email.c_str());
                cout << "[" << ret.code << "] " << ret.msg << endl;
                if (ret.code != 0) break;

                string pwd, code;
                cout << "请输入密码："; cin >> pwd;
                cout << "请输入验证码："; cin >> code;

                auto r = user_register(email.c_str(), pwd.c_str(), code.c_str());
                cout << "[" << r.code << "] " << r.msg << endl;
                break;
            }

            // ========== 登录 ==========
            case 2: {
                string email;
                cout << "\n请输入邮箱：";
                cin >> email;

                cout << "1 发送验证码" << endl;
                cout << "0 退出" << endl;
                cout << "请选择：";
                int opt; cin >> opt; clear_cin();
                if (opt != 1) break;

                auto ret = send_email_code(email.c_str());
                cout << "[" << ret.code << "] " << ret.msg << endl;
                if (ret.code != 0) break;
                string p, code;
                cout << "请输入验证码："; cin >> code;
                cout << "密码："; cin >> p;
                auto r = user_login(email.c_str(), p.c_str(), code.c_str());
                cout << "[" << r.code << "] " << r.msg << endl;
                break;
            }

            // ========== 修改密码 ==========
            case 3: {
                string email;
                cout << "\n请输入邮箱：";
                cin >> email;

                cout << "1 发送验证码" << endl;
                cout << "0 退出" << endl;
                cout << "请选择：";
                int opt; cin >> opt; clear_cin();
                if (opt != 1) break;

                auto ret = send_email_code(email.c_str());
                cout << "[" << ret.code << "] " << ret.msg << endl;
                if (ret.code != 0) break;

                string pwd, code;
                cout << "请输入新密码："; cin >> pwd;
                cout << "请输入验证码："; cin >> code;

                auto r = reset_pwd(email.c_str(), pwd.c_str(), code.c_str());
                cout << "[" << r.code << "] " << r.msg << endl;
                break;
            }

            // ========== 接口调用 ==========
            case 4: {
                auto r = call_api();
                cout << "[" << r.code << "] " << r.msg << endl;
                if (r.code == 0) {
                    cout << "✅ 已使用：" << r.used
                         << " ｜ 剩余：" << r.remain
                         << " ｜ 总次数：" << r.used + r.remain << endl;
                }
                break;
            }

            // ========== 检测 token ==========
            case 5: {
                cout << "\n正在检测本地token是否有效..." << endl;
                CheckTokenResponse r = check_token();
                cout << "[" << r.code << "] " << r.msg << endl;

                if (r.code == 0) {
                    cout << "✅ 当前登录邮箱：" << r.email << endl;
                    cout << "✅ 用户角色：" << r.role << endl;
                }
                break;
            }

            case 0:
                cout << "已退出" << endl;
                return 0;

            default:
                cout << "无效选项" << endl;
        }
    }
}