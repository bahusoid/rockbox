#ifndef __HIBY_BLUETOOTH_H__
#define __HIBY_BLUETOOTH_H__

#include <stdbool.h>

int hiby_bluetooth_menu(void);

#if defined(HAVE_HIBY_BLUETOOTH)
bool bt_is_enabled_fast(void);
bool bt_disable(void);
bool bt_enable(void);
bool bt_is_connected_fast(void);
bool bt_autoconnection_route_to_bluetooth(char active_mac[18], bool bt_on);
void bt_route_to_local(void);
#endif

#endif /* __HIBY_BLUETOOTH_H__ */
