/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#include <rgbdGrabber/realSenseGrabber.hpp>

int main (int argc, char** argv)
{
  rgbdGrabber::RealSenseGrabber g(640,480,60);
  g.run ();
  return (0);
}
