/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#include <rgbdGrabber/openni2Grabber.hpp>


int main (int argc, char** argv)
{
  rgbdGrabber::Openni2Grabber g(640,480,60);
  g.run ();
  return (0);
}

