package com.example.joystickcontroller.ui

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.min

class GameButton @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var buttonText = ""
    private var isPressed = false
    private var buttonCallback: ((pressed: Boolean) -> Unit)? = null
    private var buttonColor = Color.GRAY  // Default color

    private val buttonPaint = Paint().apply {
        isAntiAlias = true
        style = Paint.Style.FILL
    }

    private val textPaint = Paint().apply {
        isAntiAlias = true
        color = Color.WHITE
        textAlign = Paint.Align.CENTER
        textSize = 40f  // Default text size
        typeface = Typeface.DEFAULT_BOLD
    }

    private val borderPaint = Paint().apply {
        isAntiAlias = true
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }

    fun setText(text: String) {
        buttonText = text
        invalidate()
    }

    fun setButtonColor(color: Int) {
        buttonColor = color
        invalidate()
    }

    fun setTextSize(size: Float) {
        textPaint.textSize = size
        invalidate()
    }

    fun setButtonCallback(callback: (pressed: Boolean) -> Unit) {
        buttonCallback = callback
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val centerX = width / 2f
        val centerY = height / 2f
        val radius = min(width, height) / 2f - 5f

        // Set button color based on pressed state
        buttonPaint.color = if (isPressed) {
            // Darker version when pressed
            Color.rgb(
                (Color.red(buttonColor) * 0.7f).toInt(),
                (Color.green(buttonColor) * 0.7f).toInt(),
                (Color.blue(buttonColor) * 0.7f).toInt()
            )
        } else {
            buttonColor
        }

        // Draw button circle
        canvas.drawCircle(centerX, centerY, radius, buttonPaint)
        
        // Draw border
        canvas.drawCircle(centerX, centerY, radius, borderPaint)

        // Draw text
        if (buttonText.isNotEmpty()) {
            val textY = centerY - (textPaint.descent() + textPaint.ascent()) / 2
            canvas.drawText(buttonText, centerX, textY, textPaint)
        }
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                isPressed = true
                buttonCallback?.invoke(true)
                invalidate()
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                isPressed = false
                buttonCallback?.invoke(false)
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }
}