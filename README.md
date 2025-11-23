# STM32 + MPU6050

Projekt do obsługi czujnika MPU6050 na STM32. Robi odczyty z akcelerometru i żyroskopu, liczy pozycję w przestrzeni 3D i wysyła to wszystko przez UART.

## Co tu jest

Projekt powstał jako część większego systemu do monitorowania ruchu. Główne funkcje:
- Ciągły odczyt danych z MPU6050 (akcelerometr + żyroskop)
- Bufory na ~700 próbek z każdego czujnika
- Obliczanie pozycji w 3D na podstawie filtru komplementarnego
- Komunikacja przez UART z własnym protokołem (CRC8)
- Możliwość zmiany czułości i częstotliwości próbkowania

Kod jest podzielony na moduły żeby łatwiej się w tym poruszać - każda funkcjonalność ma swój plik.

## Wymagania

- STM32CubeIDE 
- Płytka ze STM32 (u mnie F3, ale powinno działać na innych)
- Czujnik MPU6050 
- Kabel USB do wgrywania
- Terminal (PuTTY, HTerm, cokolwiek)

## Struktura

```
Core/
├── Inc/
│   ├── mpu6050.h       -> wszystko związane z czujnikiem
│   ├── uart_comm.h     -> komunikacja UART (buforowana)
│   ├── protocol.h      -> parsowanie ramek, CRC
│   └── commands.h      -> obsługa komend
└── Src/
    ├── main.c          -> inicjalizacja + główna pętla
    └── [reszta .c]
```

## Jak podłączyć MPU6050

Standardowe połączenie I2C:

```
MPU6050          STM32
  VCC     ->     3.3V
  GND     ->     GND  
  SCL     ->     PB8 (I2C1)
  SDA     ->     PB9 (I2C1)
```

Adres I2C: 0x68 (domyślny)

## Uruchamianie

1. Otwórz `.ioc` w STM32CubeIDE (wygeneruje potrzebne pliki)
2. Build projektu
3. Wgraj na płytkę
4. Podłącz terminal: **115200 baud, 8N1**
5. Wyślij jakąś komendę żeby sprawdzić czy działa

## Komendy

Format ramki: `:SRC_ADDDST_ADD000CMD_NAMEDATA;`

Przykłady:
- `ACCB 1` - włącza ciągły odczyt z akcelerometru
- `ZYRB 1` - włącza ciągły odczyt z żyroskopu  
- `GPOZ 1` - włącza obliczanie pozycji 3D
- `CACC 4` - ustawia zakres akcelerometru na ±4g
- `CZYR 500` - ustawia zakres żyroskopu na ±500°/s
- `TUSP 0` - wyłącza tryb uśpienia czujnika
- `ACCA 50` - wyświetla ostatnie 50 pomiarów z akcelerometru
- `POBR 1000` - ustawia częstotliwość wysyłania danych (ms)

Parametr 0 wyłącza daną funkcję, 1 włącza (dla funkcji ciągłych).

## Jak to działa

**Timery:**
- TIM1/TIM2 - próbkowanie czujników (domyślnie 100ms)
- TIM6 - wysyłanie danych (domyślnie 1s)

**Bufory:**
- Bufor kołowy na 700 pomiarów dla akcelerometru
- Bufor kołowy na 700 pomiarów dla żyroskopu
- Można pytać o dane historyczne

**Pozycja 3D:**
Używam filtru komplementarnego (alpha=0.98) żeby połączyć dane z żyroskopu i akcelerometru. Pozycja liczona przez całkowanie przyśpieszenia z odjętą grawitacją. Nie jest idealne (drift po czasie) ale działa okej do krótkich pomiarów.

## Uwagi

- Pierwszy odczyt po resecie może być dziwny, daj chwilę na ustabilizowanie
- Obliczanie pozycji 3D kumuluje błędy, najlepiej resetować co jakiś czas
- Protokół wymaga poprawnego CRC inaczej rzuci błąd
- Bufory są volatile więc odczyt jest thread-safe (przerwania)

## Licencja

Kod bazowy od STMicroelectronics, reszta wykonana przeze mnie.

---

Jakby coś nie działało albo masz pytania to pisz issue.
