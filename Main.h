#ifndef _Main_H_
#define _Main_H_

#ifdef __cplusplus
extern "C" {
#endif
	
	struct Accel
	{
		float x,y,z;
	};

	struct Gyro
	{
		float x,y,z;
	};
	
	struct Mag
	{
		float x,y,z;
	};
	
	struct Angle
	{
		float pitch, roll, yaw;
	};
	
#ifdef __cplusplus
}
#endif

#endif /* __Main_H */
