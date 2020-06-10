# The EYELib

A library for unifying a clean and straight forward implementation -in matlab
and python- of the most common (and not so common) algorithms and techniques 
used for Single View Metrology and one image self/auto calibration.

These algorithms are broadly explained in advanced courses of computer 
vision, books and papers, however, open source implementations of such 
algorithms are very scarce or simply don't exist. 

The library depends heavily on vanishing points. The stress is on what can 
be done with the vanishing points not on obtaining them.  The EYELib assumes 
that vanishing points are provided by a third party library. 

Follows a list of stuff implemented so far (trying to keep this updated):
- Implementation of the first part of Abbas and Zisserman ICCVW19 paper 
"A Geometric Approach to Obtain a Bird’s Eye View from an Image"
- Attempt to implement 4-D SCENE ALIGNMENT IN SURVEILLANCE VIDEO by
RobertWagner, Daniel Crispell, Patrick Feeney, Joe Mundy
- Implementation of pitch angle angle calculation using the Image of the 
Absolute Conic. See "Multiple view Geometry in Computer Vision" by Harley 
and Zissermanequation 8.9 pag 210 Second Edition.
- Implementation of algorithm "Calibration from three orthogonal vanishing 
points".  See "Multiple view Geometry in Computer Vision" by Harley and 
Zisserman Example 8.27 pag 225-226 Second Edition.
- Implementation of camera height calculation. SeeCriminisi, A., Reid, I. &
 Zisserman, A. Single View Metrology. International Journal of Computer 
Vision 40, 123–148 (2000). https://doi.org/10.1023/A:1026598000963
- Implementation of rotation matrix from vanishing points and viceversa.

## Dependencies

- The library depends frequently on Peter Corke toolboxes
P.I. Corke, “Robotics, Vision & Control”, Springer 2011, ISBN 978-3-642-20143-1. **Important: notice that the paths to these toolboxes must be inserted on top of your matlab "setpath". Otherwise, there are some conflics with preessinting matlab rutines. Tested with Machine Vision toolbox on top followed by Robotics Toolbox**
- The library also depends on the David Legland geom3d library:
David Legland (2020). geom3d (https://www.mathworks.com/matlabcentral/fileexchange/24484-geom3d), MATLAB Central File Exchange. Retrieved May 26, 2020.

## Disclaimer

Notice that, in principle, the library is not dealing with distortion or image noise issues. For testing, the library will be using synthetic images or ideal (very low distortion-noisy) images.
Actually, the final purpose of the library is to provide a semi-automatic or eventually automatic way of extracting the geometry from a single image. Notice that this will highly depend on the quality of vanishing points or distortion estimation based on third party libraries. Ideally, in some near future the library may converge into deep learning approaches.
