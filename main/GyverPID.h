/*
    GyverPID - библиотека PID регулятора для Arduino
    Документация: https://alexgyver.ru/gyverpid/
    GitHub: https://github.com/GyverLibs/GyverPID
    Возможности:
    - Время одного расчёта около 70 мкс
    - Режим работы по величине или по её изменению (для интегрирующих процессов)
    - Возвращает результат по встроенному таймеру или в ручном режиме
    - Встроенные калибровщики коэффициентов
    - Режим работы по ошибке и по ошибке измерения
    - Встроенные оптимизаторы интегральной суммы

    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License

    Версии:
    v1.1 - убраны дефайны
    v1.2 - возвращены дефайны
    v1.3 - вычисления ускорены, библиотека облегчена
    v2.0 - логика работы чуть переосмыслена, код улучшен, упрощён и облегчён
    v2.1 - integral вынесен в public
    v2.2 - оптимизация вычислений
    v2.3 - добавлен режим PID_INTEGRAL_WINDOW
    v2.4 - реализация внесена в класс
    v3.0
        - Добавлен режим оптимизации интегральной составляющей (см. доку)
        - Добавлены автоматические калибровщики коэффициентов (см. примеры и доку)
    v3.1 - исправлен режиме ON_RATE, добавлено автоограничение инт. суммы
    v3.2 - чуть оптимизации, добавлена getResultNow
    v3.3 - в тюнерах можно передать другой обработчик класса Stream для отладки
*/

#ifndef _GyverPID_h
#define _GyverPID_h

#pragma once

#include "string.h"
#include "PIDTuner.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nvs.h"

#if defined(PID_INTEGER)  // расчёты с целыми числами
typedef int16_t pidtype;
#else  // расчёты с float числами
typedef float pidtype;
#endif

#define NORMAL 0
#define REVERSE 1
#define ON_ERROR 0
#define ON_RATE 1

static float constrain(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static bool get_coef(nvs_handle_t handle, const char* name, float* coef) {
    esp_err_t err = nvs_get_i32(handle, name, (int32_t*)coef);
    return ESP_OK == err;
}

class GyverPID {
   public:
    // ==== pidtype это float или int, в зависимости от выбранного (см. пример integer_calc) ====
    GyverPID() {}

    // kp, ki, kd, dt
    GyverPID(const char* pid_name, uint32_t new_dt = 100) {
        nvs_handle_t handle;
        esp_err_t err = nvs_open(pid_name, NVS_READWRITE, &handle);
        pid_name_ = pid_name;
        calibration_mode = !(get_coef(handle, "kp", &Kp) && get_coef(handle, "ki", &Ki) && get_coef(handle, "kd", &Kd));
        nvs_close(handle);
        
        if(calibration_mode) {
            ESP_LOGE("PID", "Error occured while reading PID params for %s ", pid_name);
        }
        setDt(new_dt);
    }

    // направление регулирования: NORMAL (0) или REVERSE (1)
    void setDirection(bool direction) {
        _direction = direction;
    }

    // режим: работа по входной ошибке ON_ERROR (0) или по изменению ON_RATE (1)
    void setMode(bool mode) {
        _mode = mode;
    }

    // лимит выходной величины (например для ШИМ ставим 0-255)
    void setLimits(float min_output, float max_output) {
        _minOut = min_output;
        _maxOut = max_output;
    }

    // установка времени дискретизации (для getResultTimer)
    void setDt(uint32_t new_dt) {
        _dt_s = new_dt / 1000.0f;
        _dt = new_dt;
    }

    pidtype setpoint = 0;  // заданная величина, которую должен поддерживать регулятор
    pidtype input = 0;     // сигнал с датчика (например температура, которую мы регулируем)
    pidtype output = 0;    // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
    float Kp = 0.0;        // коэффициент P
    float Ki = 0.0;        // коэффициент I
    float Kd = 0.0;        // коэффициент D
    float integral = 0.0;  // интегральная сумма
    float _minOut = 0, _maxOut = 255;
    PIDtuner tuner_;

    // возвращает новое значение при вызове (если используем свой таймер с периодом dt!)
    pidtype getResult() {
        if(calibration_mode) {
            if(strcmp(pid_name_, "lidar") == 0) {
                ESP_LOGW(pid_name_ , "CALIB %f accuracy %d", input, tuner_.getAccuracy());
                tuner_.print_parameters();
            }
            
            tuner_.setInput(input);
            tuner_.compute();
            if(tuner_.getAccuracy() > 90) {
                nvs_handle_t handle;
                esp_err_t err = nvs_open(pid_name_, NVS_READWRITE, &handle);
                int32_t kp_int, ki_int, kd_int;
                float kp = tuner_.getPID_p();
                float ki = tuner_.getPID_i();
                float kd = tuner_.getPID_d();
                memcpy(&kp_int, &kp, sizeof(int32_t));
                memcpy(&ki_int, &ki, sizeof(int32_t));
                memcpy(&kd_int, &kd, sizeof(int32_t));
                nvs_set_i32(handle, "kp", kp_int);
                nvs_set_i32(handle, "ki", ki_int);
                nvs_set_i32(handle, "kd", kd_int);


                nvs_commit(handle);
                nvs_close(handle);
                ESP_LOGW("PID", "successfully calibrated %s", pid_name_);
                calibration_mode = false;
            }
            ESP_LOGI(pid_name_, "value %f", tuner_.getOutput());
            output = constrain(tuner_.getOutput(), _minOut, _maxOut);
            return output;
        }
        pidtype error = setpoint - input;         // ошибка регулирования
        pidtype delta_input = prevInput - input;  // изменение входного сигнала за dt
        prevInput = input;                        // запомнили предыдущее
        if (_direction) {                         // смена направления
            error = -error;
            delta_input = -delta_input;
        }
        output = _mode ? 0 : (error * Kp);   // пропорциональая составляющая
        output += delta_input * Kd / _dt_s;  // дифференциальная составляющая

#if (PID_INTEGRAL_WINDOW > 0)
        // ЭКСПЕРИМЕНТАЛЬНЫЙ РЕЖИМ ИНТЕГРАЛЬНОГО ОКНА
        if (++t >= PID_INTEGRAL_WINDOW) t = 0;  // перемотка t
        integral -= errors[t];                  // вычитаем старое
        errors[t] = error * Ki * _dt_s;         // запоминаем в массив
        integral += errors[t];                  // прибавляем новое
#else
        integral += error * Ki * _dt_s;  // обычное суммирование инт. суммы
#endif

#ifdef PID_OPTIMIZED_I
        // ЭКСПЕРИМЕНТАЛЬНЫЙ РЕЖИМ ОГРАНИЧЕНИЯ ИНТЕГРАЛЬНОЙ СУММЫ
        output = constrain(output, _minOut, _maxOut);
        if (Ki != 0) integral = constrain(integral, (_minOut - output) / (Ki * _dt_s), (_maxOut - output) / (Ki * _dt_s));
#endif

        if (_mode) integral += delta_input * Kp;           // режим пропорционально скорости
        integral = constrain(integral, _minOut, _maxOut);  // ограничиваем инт. сумму
        output += integral;                                // интегральная составляющая
        output = constrain(output, _minOut, _maxOut);      // ограничиваем выход
        return output;
    }

    // возвращает новое значение не ранее, чем через dt миллисекунд (встроенный таймер с периодом dt)
    pidtype getResultTimer() {
        if (millis() - pidTimer >= _dt) {
            pidTimer = millis();
            getResult();
        }
        return output;
    }

    // посчитает выход по реальному прошедшему времени между вызовами функции
    pidtype getResultNow() {
        setDt(millis() - pidTimer);
        pidTimer = millis();

        return getResult();
    }

    bool calibration_mode;
   private:
    uint32_t _dt = 100;  // время итерации в мс
    float _dt_s = 0.1;   // время итерации в с
    bool _mode = 0, _direction = 0;
    
    pidtype prevInput = 0;
    uint32_t pidTimer = 0;
    
    const char* pid_name_;
    
#if (PID_INTEGRAL_WINDOW > 0)
    pidtype errors[PID_INTEGRAL_WINDOW];
    uint16_t t = 0;
#endif
};
#endif