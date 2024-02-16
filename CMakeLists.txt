cmake_minimum_required(VERSION 3.13)

project(motion.emd.icm45686.driver)

function(configure_executable _name)
	message(STATUS "Configuring ${_name}")
	add_executable(${_name} 
	               ${CMAKE_CURRENT_SOURCE_DIR}/examples/${_name}/${_name}.c
	               ${_sources})

	target_include_directories(${_name} PRIVATE
	                           ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/
	                           ${CMAKE_CURRENT_SOURCE_DIR}/examples/)
endfunction()

set(_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver_advanced.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver_advanced.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver_aux1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_driver_aux1.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_edmp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_edmp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_edmp_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_edmp_memmap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_regmap_be.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_regmap_le.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_selftest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_selftest.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_transport.h
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686/imu/inv_imu_version.h
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/system_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/system_interface.c)

configure_executable(basic_read_fifo)
configure_executable(basic_read_registers)
configure_executable(adv_read_fifo)
configure_executable(adv_fifo_compression)
configure_executable(adv_wom)
configure_executable(adv_fsync)
configure_executable(aux1)
configure_executable(edmp_apex)
configure_executable(selftest)

