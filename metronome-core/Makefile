PROJECT_NAME := metronome

include $(IDF_PATH)/make/project.mk

ifdef CONFIG_M_DATA_DEPLOY_SF
	DATA_SRC_DIR = lol
ifneq ($(wildcard $(DATA_SRC_DIR)/*),)

	SPIFFS_IMAGE_FLASH_IN_PROJECT := 1

	@echo ====== Generating SPIFFS image of <$(DATA_SRC_DIR)> ======
	@echo Mount point in VFS : $(CONFIG_M_DATA_MOUNT_POINT)
	$(eval $(call spiffs_create_partition_image,$(CONFIG_M_DATA_MOUNT_POINT),$(DATA_SRC_DIR)))
	@echo SPIFFS image renerated !
else
	@echo Could not generate SPIFFS image !
	$(error $(DATA_SRC_DIR) doesn't exist.)
endif
endif
