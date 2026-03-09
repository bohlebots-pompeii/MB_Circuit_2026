//
// Created by julius on 11.11.2025.
//

#ifndef BOHLEBOTS_2026_DEBUG_H
#define BOHLEBOTS_2026_DEBUG_H

#include <string>

class Log {
public:
    static void header();

    static void debug(const std::string& _i);

    static void info(const std::string& _i);

    static void warning(const std::string& _i);

    static void error(const std::string& _i);
};
#endif //BOHLEBOTS_2026_DEBUG_H