#ifndef LOG_H
#define LOG_H

#define RESET         "\033[0m"
#define BOLD_RED      "\033[1;31m"
#define BOLD_GREEN    "\033[1;32m"
#define BOLD_BLUE     "\033[1;34m"
#define BOLD_CYAN     "\033[1;36m"
#define GREY          "\033[90m"

/* [RTL-SDR] / [BladeRF] tag — bold blue */
#define TAG_RTL    BOLD_BLUE "[RTL-SDR]" RESET
#define TAG_BLADE  BOLD_BLUE "[BladeRF]" RESET

#define LOG_HEADER(fmt, ...)  printf(BOLD_CYAN fmt RESET "\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    (printf(GREY fmt RESET, ##__VA_ARGS__), fflush(stdout))
#define LOG_OK(fmt, ...)      printf(BOLD_GREEN fmt RESET,      ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)     fprintf(stderr, BOLD_RED fmt RESET, ##__VA_ARGS__)

#endif /* LOG_H */
