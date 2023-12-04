/*uint8_t b;
    if (Serial.available()) {
      
      b = Serial.read();    

      if (b == 'a'){
        Servo1_DC -= 100; 
      }
      else if (b == 'q'){
        Servo1_DC += 100;
      }
      else if (b == 's'){
        Servo2_DC -= 0.01;
      }
      else if (b == 'w'){
        Servo2_DC += 0.01;
      }
      else serial_commands.process_char(b);

      if (Servo1_DC > 13.2) Servo1_DC = 13.2;
      if (Servo1_DC < 2.90) Servo1_DC = 2.90;

      if (Servo2_DC > 20) Servo2_DC = 20;
      if (Servo2_DC < 0) Servo2_DC = 0;
    }
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;

      show_lux = 1;
      uint16_t r, g, b, c, colorTemp, lux;
      getRawData_noDelay(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      if (show_lux) lux = tcs.calculateLux(r, g, b);

      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      if (show_lux) Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
      Serial.print("\n");
      
      // Update the LED intensity
      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);


      // Debug using the serial port
      Serial.print("DC Servo1 : ");
      Serial.print(Servo1_DC);
      Serial.print("\n");
      Serial.print("DC Servo2 : ");
      Serial.print(Servo2_DC);
      Serial.print("\n");

      if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
      }
      tof.startReadRangeMillimeters(); 
      Serial.print("Dist: ");
      Serial.print(distance, 3);
      Serial.println();
    }*/