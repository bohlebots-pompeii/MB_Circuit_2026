//
// Created by julius on 11.11.2025.
//

#include <iostream>
#include "../../include/util/log.h"

void Log::header() {
    std::cout << "Pompeii - 2026 - RCJS 2v2" << std::endl;
}

void Log::debug(const std::string& _i) {
    std::cout << "[DEBUG] " + _i << std::endl;
}

void Log::info(const std::string& _i) {
    std::cout << "[INFO] " + _i << std::endl;
}

void Log::warning(const std::string& _i) {
    std::cout << "[WARN] " + _i << std::endl;

}

void Log::error(const std::string& _i) {
    std::cout << "[ERROR] " + _i << std::endl;

}
