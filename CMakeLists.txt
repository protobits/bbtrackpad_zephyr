if (CONFIG_BBTRACKPAD_DRIVER)
	zephyr_library()
	zephyr_library_sources(trackpad.c)
endif()
