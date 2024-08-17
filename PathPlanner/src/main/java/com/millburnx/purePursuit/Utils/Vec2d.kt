package com.millburnx.purePursuit.Utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Represents a 2D vector/point
 */
class Vec2d(val x: Double, val y: Double) {
    constructor(point: java.awt.Point) : this(point.x.toDouble(), point.y.toDouble())

    operator fun plus(other: Vec2d) = Vec2d(x + other.x, y + other.y)
    operator fun plus(other: Double) = Vec2d(x + other, y + other)
    operator fun plus(other: Float) = this + other.toDouble()
    operator fun plus(other: Int) = this + other.toDouble()

    operator fun minus(other: Vec2d) = Vec2d(x - other.x, y - other.y)
    operator fun minus(other: Double) = Vec2d(x - other, y - other)
    operator fun minus(other: Float) = this - other.toDouble()
    operator fun minus(other: Int) = this - other.toDouble()

    operator fun times(other: Vec2d) = Vec2d(x * other.x, y * other.y)
    operator fun times(other: Double) = Vec2d(x * other, y * other)
    operator fun times(other: Float) = this * other.toDouble()
    operator fun times(other: Int) = this * other.toDouble()

    operator fun div(other: Vec2d) = Vec2d(x / other.x, y / other.y)
    operator fun div(other: Double) = Vec2d(x / other, y / other)
    operator fun div(other: Float) = this / other.toDouble()
    operator fun div(other: Int) = this / other.toDouble()

    operator fun unaryMinus() = Vec2d(-x, -y)

    /**
     * Returns the euclidean distance to another point
     */
    fun distanceTo(other: Vec2d): Double {
        val xDiff = x - other.x
        val yDiff = y - other.y
        return sqrt(xDiff * xDiff + yDiff * yDiff)
    }

    /**
     * Returns the angle to another point
     */
    fun angleTo(other: Vec2d): Double {
        return atan2(other.y - y, other.x - x)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Vec2d) return false
        return x == other.x && y == other.y
    }

    override fun toString(): String {
        return "Point($x, $y)"
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    /**
     * Rotates the vector by an angle
     */
    fun rotate(angle: Double): Vec2d {
        val cos = cos(angle)
        val sin = sin(angle)
        return Vec2d(x * cos - y * sin, x * sin + y * cos)
    }

    /**
     * Linearly interpolates between two vectors/points
     */
    fun lerp(other: Vec2d, t: Double) = this + (other - this) * t
    fun lerp(other: Vec2d, t: Float) = this.lerp(other, t.toDouble())

    /**
     * Converts the point to an java.awt point
     */
    fun awt(): java.awt.Point {
        return java.awt.Point(x.toInt(), y.toInt())
    }

    /**
     * Returns a copy of the point
     */
    fun copy(): Vec2d {
        return Vec2d(x, y)
    }

    companion object {
        /**
         * Saves a list of points to a tsv file
         */
        fun saveList(points: List<Vec2d>, file: java.io.File) {
            val data = points.map { listOf(it.x.toString(), it.y.toString()) }
            TSV.bufferedWrite(file, data)
        }

        /**
         * Loads a list of points from a tsv file
         */
        fun loadList(file: java.io.File): List<Vec2d> {
            val data = TSV.bufferedRead(file)
            return data.map { Vec2d(it[0].toDouble(), it[1].toDouble()) }
        }
    }
}