#ifndef _APP_VERSION_H_
#define _APP_VERSION_H_

#define APP_VERSION_MAJOR      (1u)
#define APP_VERSION_MINOR      (0u)
#define APP_VERSION_PATCH      (0u)
#define APP_VERSION_BUILD      (85u)

#define APP_VERSION_NUM        ((APP_VERSION_MAJOR << 28u) | (APP_VERSION_MINOR << 24u) | \
        (APP_VERSION_PATCH << 16u) | (APP_VERSION_BUILD))

#endif /* _APP_VERSION_H_ */

