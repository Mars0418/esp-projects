#pragma once
#include <stdbool.h>
#include <string.h>

/* Match a complete instruction, never a substring in a question/negation. */
static inline bool dialogue_is_end_request(const char *text)
{
    static const char *phrases[] = {
        "结束对话", "停止对话", "退出对话", "乐迪结束对话", "乐迪，结束对话"
    };
    if (!text) return false;
    while (*text == ' ' || *text == '\t') ++text;
    for (unsigned i = 0; i < sizeof(phrases) / sizeof(phrases[0]); ++i) {
        size_t n = strlen(phrases[i]);
        if (strncmp(text, phrases[i], n)) continue;
        const char *tail = text + n;
        while (*tail) {
            if (strchr(" \t\r\n.!?,", *tail)) { ++tail; continue; }
            if (!strncmp(tail, "。", 3) || !strncmp(tail, "！", 3) ||
                !strncmp(tail, "？", 3) || !strncmp(tail, "，", 3)) { tail += 3; continue; }
            break;
        }
        if (!*tail) return true;
    }
    return false;
}
