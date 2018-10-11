# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
obj-y   +=  gyroscope.o gyro_factory.o

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_GYROHUB)      +=  gyrohub/
else
obj-$(CONFIG_MTK_LSM6DS3HGY)   +=  lsm6ds3hgy/
# [yanlin start] Porting sensors
obj-$(CONFIG_MTK_BMI160_GYRO)   +=  bmi160_gyro/
obj-$(CONFIG_MTK_LSM6DS3_GYRO)   +=  lsm6ds3-gyro/
# [yanlin end]
endif
