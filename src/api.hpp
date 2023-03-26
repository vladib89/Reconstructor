#ifndef API_HPP
#define API_HPP

#if defined(WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#elif defined(UNIX)
/* Do linux stuff */
#else
#endif

#endif // !API_HPP
