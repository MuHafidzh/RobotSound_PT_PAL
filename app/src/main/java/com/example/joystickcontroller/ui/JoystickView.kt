package com.example.joystickcontroller.ui

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.*

class JoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var centerX = 0f
    private var centerY = 0f
    private var baseRadius = 0f
    private var knobRadius = 0f
    private var knobX = 0f
    private var knobY = 0f

    private val basePaint = Paint().apply {
        isAntiAlias = true
        color = Color.rgb(70, 70, 70) // Dark gray base
        style = Paint.Style.FILL
    }

    private val knobPaint = Paint().apply {
        isAntiAlias = true
        color = Color.rgb(100, 100, 100) // Lighter gray knob
        style = Paint.Style.FILL
    }

    private val borderPaint = Paint().apply {
        isAntiAlias = true
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private var joystickCallback: ((x: Float, y: Float) -> Unit)? = null

    fun setJoystickCallback(callback: (x: Float, y: Float) -> Unit) {
        joystickCallback = callback
    }
    
    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        
        centerX = w / 2f
        centerY = h / 2f
        
        // Calculate knob radius
        knobRadius = min(w, h) * 0.12f  // Slightly bigger knob
        
        // Allow knob to extend half outside - reduce padding
        val padding = knobRadius / 2f  // Only half knob radius padding
        baseRadius = (min(w, h) / 2f) - padding
        
        // Initialize knob position to center
        knobX = centerX
        knobY = centerY
    }
    
    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.action) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                val touchX = event.x
                val touchY = event.y
                
                // Calculate distance from center
                val distance = sqrt((touchX - centerX).pow(2) + (touchY - centerY).pow(2))
                
                // Allow knob to move to full base radius (knob can extend half outside)
                if (distance <= baseRadius) {
                    knobX = touchX
                    knobY = touchY
                } else {
                    // Constrain to base radius edge
                    val angle = atan2(touchY - centerY, touchX - centerX)
                    knobX = centerX + cos(angle) * baseRadius
                    knobY = centerY + sin(angle) * baseRadius
                }
                
                // Calculate normalized values using base radius
                val normalizedX = -((knobX - centerX) / baseRadius)
                val normalizedY = -((knobY - centerY) / baseRadius)
                
                // Clamp values to ensure they stay within [-1, 1]
                val clampedX = normalizedX.coerceIn(-1f, 1f)
                val clampedY = normalizedY.coerceIn(-1f, 1f)
                
                joystickCallback?.invoke(clampedX, clampedY)
                invalidate()
                return true
            }
            
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                // Return knob to center
                knobX = centerX
                knobY = centerY
                joystickCallback?.invoke(0f, 0f)
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }
    
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
    
        // Draw outer circle (base)
        canvas.drawCircle(centerX, centerY, baseRadius, basePaint)
        canvas.drawCircle(centerX, centerY, baseRadius, borderPaint)
    
        // Draw center crosshair for reference
        val crossSize = 8f
        borderPaint.strokeWidth = 2f
        canvas.drawLine(centerX - crossSize, centerY, centerX + crossSize, centerY, borderPaint)
        canvas.drawLine(centerX, centerY - crossSize, centerX, centerY + crossSize, borderPaint)
        borderPaint.strokeWidth = 3f
    
        // Draw knob (can extend outside base circle)
        canvas.drawCircle(knobX, knobY, knobRadius, knobPaint)
        canvas.drawCircle(knobX, knobY, knobRadius, borderPaint)
    }
}