# Is this the code that takes care of the depth calibration for Kinect in glview.c?

```
int i;
for (i=0; i<2048; i++) {
	float v = i/2048.0;
	v = powf(v, 3)* 6;
	t_gamma[i] = v*6*256;
}
```

# No, the one in glview.c is wrong. Here is a better one (slightly better than the 1/x one):

```
for (size_t i=0; i<2048; i++)
{
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;
	const float depth = k3 * tanf(i/k2 + k1);
	t_gamma[i] = depth;
}
```

# Raw depth values are integer between 0 and 2047. They can be transformed into depth in meters using the parameters given on the Ros Kinect page.

```
float raw_depth_to_meters(int raw_depth)
{
  if (raw_depth < 2047)
  {
   return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
  }
  return 0;
}
```

# Mapping depth pixels with color pixels

https://nicolas.burrus.name/oldstuff/kinect_calibration/

The first step is to undistort rgb and depth images using the estimated distortion coefficients. Then, using the depth camera intrinsics, each pixel (x_d,y_d) of the depth camera can be projected to metric 3D space using the following formula:  

```
P3D.x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
P3D.y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
P3D.z = depth(x_d,y_d)
```

