/* -*- mode: java; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Processing project - http://processing.org

  Copyright (c) 2012-17 The Processing Foundation
  Copyright (c) 2008-12 Ben Fry and Casey Reas
  Copyright (c) 2008 Dan Shiffman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
 */

//package processing.core;
package org.firstinspires.ftc.teamcode;
import java.io.Serializable;


/**
 * NOTE: despite the massive comment below this one saying that this is
 * the processing PVector class, it at this point is not. many sections
 * of the original code have been deleted or commented out and everything
 * *should* be using doubles instead of floats now to better integrate with
 * java.lang.Math instead of processing's PApplet functions.
 */

/**
 * ( begin auto-generated from PVector.xml )
 *
 * A class to describe a two or three dimensional vector. This datatype
 * stores two or three variables that are commonly used as a position,
 * velocity, and/or acceleration. Technically, <em>position</em> is a point
 * and <em>velocity</em> and <em>acceleration</em> are vectors, but this is
 * often simplified to consider all three as vectors. For example, if you
 * consider a rectangle moving across the screen, at any given instant it
 * has a position (the object's location, expressed as a point.), a
 * velocity (the rate at which the object's position changes per time unit,
 * expressed as a vector), and acceleration (the rate at which the object's
 * velocity changes per time unit, expressed as a vector). Since vectors
 * represent groupings of values, we cannot simply use traditional
 * addition/multiplication/etc. Instead, we'll need to do some "vector"
 * math, which is made easy by the methods inside the <b>PVector</b>
 * class.<br />
 * <br />
 * The methods for this class are extensive. For a complete list, visit the
 * <a
 * href="http://processing.googlecode.com/svn/trunk/processing/build/javadoc/core/">developer's reference.</a>
 *
 * ( end auto-generated )
 *
 * A class to describe a two or three dimensional vector.
 * <p>
 * The result of all functions are applied to the vector itself, with the
 * exception of cross(), which returns a new PVector (or writes to a specified
 * 'target' PVector). That is, add() will add the contents of one vector to
 * this one. Using add() with additional parameters allows you to put the
 * result into a new PVector. Functions that act on multiple vectors also
 * include static versions. Because creating new objects can be computationally
 * expensive, most functions include an optional 'target' PVector, so that a
 * new PVector object is not created with each operation.
 * <p>
 * Initially based on the Vector3D class by <a href="http://www.shiffman.net">Dan Shiffman</a>.
 *
 * @webref math
 */
public class PVector implements Serializable {
    /**
     * ( begin auto-generated from PVector_x.xml )
     * <p>
     * The x component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     * <p>
     * ( end auto-generated )
     *
     * @webref pvector:field
     * @usage web_application
     * @brief The x component of the vector
     */
    public double x;

    /**
     * ( begin auto-generated from PVector_y.xml )
     * <p>
     * The y component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     * <p>
     * ( end auto-generated )
     *
     * @webref pvector:field
     * @usage web_application
     * @brief The y component of the vector
     */
    public double y;

    /**
     * ( begin auto-generated from PVector_z.xml )
     * <p>
     * The z component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     * <p>
     * ( end auto-generated )
     *
     * @webref pvector:field
     * @usage web_application
     * @brief The z component of the vector
     */
    public double z;


    /**
     * Constructor for an empty vector: x, y, and z are set to 0.
     */
    public PVector() {
    }


    /**
     * Constructor for a 3D vector.
     *
     * @param x the x coordinate.
     * @param y the y coordinate.
     * @param z the z coordinate.
     */
    public PVector(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public PVector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }


    public PVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * ( begin auto-generated from PVector_set.xml )
     * <p>
     * Sets the x, y, and z component of the vector using two or three separate
     * variables, the data from a PVector, or the values from a float array.
     * <p>
     * ( end auto-generated )
     *
     * @param x the x component of the vector
     * @param y the y component of the vector
     * @param z the z component of the vector
     * @webref pvector:method
     * @brief Set the components of the vector
     */
    public PVector set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public PVector set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }



    /**
     * ( begin auto-generated from PVector_copy.xml )
     * <p>
     * Gets a copy of the vector, returns a PVector object.
     * <p>
     * ( end auto-generated )
     *
     * @webref pvector:method
     * @usage web_application
     * @brief Get a copy of the vector
     */
    public PVector copy() {
        return new PVector(x, y, z);
    }


    @Deprecated
    public PVector get() {
        return copy();
    }


    /**
     * @param target
     */
    public double[] get(double[] target) {
        if (target == null) {
            return new double[]{x, y, z};
        }
        if (target.length >= 2) {
            target[0] = x;
            target[1] = y;
        }
        if (target.length >= 3) {
            target[2] = z;
        }
        return target;
    }


    /**
     * ( begin auto-generated from PVector_mag.xml )
     * <p>
     * Calculates the magnitude (length) of the vector and returns the result
     * as a float (this is simply the equation <em>sqrt(x*x + y*y + z*z)</em>.)
     * <p>
     * ( end auto-generated )
     *
     * @return magnitude (length) of the vector
     * @webref pvector:method
     * @usage web_application
     * @brief Calculate the magnitude of the vector
     * @see
     */
    public float mag() {
        return (float) Math.sqrt(x * x + y * y + z * z);
    }


    /**
     * Add two vectors
     *
     * @param v1 a vector
     * @param v2 another vector
     */
    static public PVector add(PVector v1, PVector v2) {
        return add(v1, v2, null);
    }


    /**
     * Add two vectors into a target vector
     *
     * @param target the target vector (if null, a new vector will be created)
     */
    static public PVector add(PVector v1, PVector v2, PVector target) {
        if (target == null) {
            target = new PVector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        } else {
            target.set(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }
        return target;
    }

    /**
     * Subtract one vector from another
     *
     * @param v1 the x, y, and z components of a PVector object
     * @param v2 the x, y, and z components of a PVector object
     */
    static public PVector sub(PVector v1, PVector v2) {
        return sub(v1, v2, null);
    }


    /**
     * Subtract one vector from another and store in another vector
     *
     * @param target PVector in which to store the result
     */
    static public PVector sub(PVector v1, PVector v2, PVector target) {
        if (target == null) {
            target = new PVector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        } else {
            target.set(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }
        return target;
    }

    /**
     * Multiply a vector by a scalar, and write the result into a target PVector.
     *
     * @param target PVector in which to store the result
     */
    static public PVector mult(PVector v, float n, PVector target) {
        if (target == null) {
            target = new PVector(v.x * n, v.y * n, v.z * n);
        } else {
            target.set(v.x * n, v.y * n, v.z * n);
        }
        return target;
    }


    /**
     * ( begin auto-generated from PVector_setMag.xml )
     * <p>
     * Calculate the angle of rotation for this vector (only 2D vectors)
     * <p>
     * ( end auto-generated )
     *
     * @return the angle of rotation
     * @webref pvector:method
     * @usage web_application
     * @brief Calculate the angle of rotation for this vector
     */
    public float heading() {
        float angle = (float) Math.atan2(y, x);
        return angle;
    }


    @Deprecated
    public float heading2D() {
        return heading();
    }


    /**
     * ( begin auto-generated from PVector_rotate.xml )
     * <p>
     * Rotate the vector by an angle (only 2D vectors), magnitude remains the same
     * <p>
     * ( end auto-generated )
     *
     * @param theta the angle of rotation
     * @webref pvector:method
     * @usage web_application
     * @brief Rotate the vector by an angle (2D only)
     */
    public PVector rotate(double theta) {
        double temp = x;
        // Might need to check for rounding errors like with angleBetween function?
        x = x * Math.cos(theta) - y * Math.sin(theta);
        y = temp * Math.sin(theta) + y * Math.cos(theta);
        return this;
    }


    @Override
    public String toString() {
        return "[ " + x + ", " + y + ", " + z + " ]";
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof PVector)) {
            return false;
        }
        final PVector p = (PVector) obj;
        return x == p.x && y == p.y && z == p.z;
    }
}