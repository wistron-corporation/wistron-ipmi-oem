# Initialization
AC_PREREQ([2.69])
AC_INIT([openpower-host-ipmi-oem], [1.0], [https://github.com/wistron-corporation/wistron-ipmi-oem/issues])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE([subdir-objects -Wall -Werror foreign dist-xz])
AM_SILENT_RULES([yes])
AC_LANG([C++])

# Checks for programs.
AC_PROG_CXX
AM_PROG_AR
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Checks for typedefs, structures, and compiler characteristics.
AX_CXX_COMPILE_STDCXX_17([noext])
AX_APPEND_COMPILE_FLAGS([-Wall -Werror], [CXXFLAGS])

# Checks for libraries.
PKG_CHECK_MODULES([LIBIPMID], [libipmid])

# Checks for library functions.
LT_INIT # Required for systemd linking

# Checks for modules
PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],, [AC_MSG_ERROR([Could not find sdbusplus...openbmc/sdbusplus package required])])
PKG_CHECK_MODULES([PHOSPHOR_LOGGING], [phosphor-logging],, [AC_MSG_ERROR([Could not find phosphor-logging...openbmc/phosphor-logging package required])])
PKG_CHECK_MODULES([PHOSPHOR_DBUS_INTERFACES], [phosphor-dbus-interfaces],, [AC_MSG_ERROR([Could not find phosphor-dbus-interfaces...openbmc/phosphor-dbus-interfaces package required])])
PKG_CHECK_MODULES([SDEVENTPLUS], [sdeventplus], [], [AC_MSG_ERROR(["sdeventplus required and not found."])])

# item interface
AC_ARG_VAR(ITEM_IFACE, [The item interface])
AS_IF([test "x$ITEM_IFACE" == "x"], [ITEM_IFACE="xyz.openbmc_project.Inventory.Item"])
AC_DEFINE_UNQUOTED([ITEM_IFACE], ["$ITEM_IFACE"], [The item interface])

# Service dbus root
AC_ARG_VAR(GPU_OBJ_PATH_ROOT, [The gpu Dbus root])
AS_IF([test "x$GPU_OBJ_PATH_ROOT" == "x"], [GPU_OBJ_PATH_ROOT="/xyz/openbmc_project/sensors/temperature"])
AC_DEFINE_UNQUOTED([GPU_OBJ_PATH_ROOT], ["$GPU_OBJ_PATH_ROOT"], [The gpu Dbus root])

# Service dbus objpath
AC_ARG_VAR(GPU_OBJ_PATH, [The gpu object path])
AS_IF([test "x$GPU_OBJ_PATH" == "x"], [GPU_OBJ_PATH="/xyz/openbmc_project/sensors/temperature/gpu"])
AC_DEFINE_UNQUOTED([GPU_OBJ_PATH], ["$GPU_OBJ_PATH"], [The gpu Dbus root])

# Dbus service name
AC_ARG_VAR(GPU_REQUEST_NAME, [The Dbus busname to own])
AS_IF([test "x$GPU_REQUEST_NAME" == "x"], [GPU_REQUEST_NAME="xyz.openbmc_project.gpu.manager"])
AC_DEFINE_UNQUOTED([GPU_REQUEST_NAME], ["$GPU_REQUEST_NAME"], [The Dbus busname to own])

# Value interface
AC_ARG_VAR(Value_IFACE, [The properies interface])
AS_IF([test "x$Value_IFACE" == "x"], [Value_IFACE="xyz.openbmc_project.Sensor.Value"])
AC_DEFINE_UNQUOTED([Value_IFACE], ["$Value_IFACE"], [The gpu Dbus root])

# Service dbus interface
AC_ARG_VAR(DBUS_PROPERTY_IFACE, [The properies interface])
AS_IF([test "x$DBUS_PROPERTY_IFACE" == "x"], [DBUS_PROPERTY_IFACE="org.freedesktop.DBus.Properties"])
AC_DEFINE_UNQUOTED([DBUS_PROPERTY_IFACE], ["$DBUS_PROPERTY_IFACE"], [The gpu Dbus root])

# Inventory dbus busname
AC_ARG_VAR(INVENTORY_BUSNAME, [The inventory busname])
AS_IF([test "x$INVENTORY_BUSNAME" == "x"], [INVENTORY_BUSNAME="xyz.openbmc_project.Inventory.Manager"])
AC_DEFINE_UNQUOTED([INVENTORY_BUSNAME], ["$INVENTORY_BUSNAME"], [The inventory busname])

# Value interface
AC_ARG_VAR(Value_IFACE, [The properies interface])
AS_IF([test "x$Value_IFACE" == "x"], [Value_IFACE="xyz.openbmc_project.Sensor.Value"])
AC_DEFINE_UNQUOTED([Value_IFACE], ["$Value_IFACE"], [The gpu Dbus root])

# Inventory path
AC_ARG_VAR(INVENTORY_PATH, [The inventory path])
AS_IF([test "x$INVENTORY_PATH" == "x"], [INVENTORY_PATH="/xyz/openbmc_project/inventory/system/chassis/motherboard/gpu"])
AC_DEFINE_UNQUOTED([INVENTORY_PATH], ["$INVENTORY_PATH"], [The inventory path])

# Inventory manager interface
AC_ARG_VAR(INVENTORY_MANAGER_IFACE, [The inventory path])
AS_IF([test "x$INVENTORY_MANAGER_IFACE" == "x"], [INVENTORY_MANAGER_IFACE="xyz.openbmc_project.Inventory.Manager"])
AC_DEFINE_UNQUOTED([INVENTORY_MANAGER_IFACE], ["$INVENTORY_MANAGER_IFACE"], [The inventory path])

# Create configured output
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
