# kalman-filter for Vehicle Localization

This implementation of Kalman Filter includes core steps implementation (`predict` and `update`) and is designed specifically for vehicle localization. 

This code depends on [Eigen library](https://d17h27t6h515a5.cloudfront.net/topher/2017/March/58b7604e_eigen/eigen.zip). I have also added this library to repo for convinience. You can also download it directly to get its latest version.

**Note:** I am using this repo only for my own learning and practice of Kalman Filter while remaining specifically in Autonomous Vehicles domain. I will keep improving it as I learns more.

# Getting Started 

For a given measurement of vehicle position `z`, the prediction and measurement update steps would look like below

```
VectorXd z = measurements[i];

// measurement update step
update(x, P, z);

// prediction step
predict(x, P);
```

