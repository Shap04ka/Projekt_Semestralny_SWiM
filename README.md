# Aby uruchomić projekt:
0) Upewnij się, że wszystkie przewody są podłączone do odpowiednich portów.
1) Podłącz przewód zasilający do płytki STM32 z komputera.
2) Skompiluj i załaduj kod z tego repozytorium na płytkę za pomocą programu STM32CubeIDE.
3) Odłącz przewód od komputera i podłącz go do powerbanku (lub innego przenośnego źródła zasilania).
4) Podłącz zestaw baterii do mostka.
5) Połącz się z modułem Bluetooth za pomocą wybranego programu, najlepiej "Serial Bluetooth Terminal".
6) Wyślij wybraną komendę (lista komend zostanie podana na końcu dokumentu).
7) Obserwuj rezultat.

*Lista komend, które można wysłać przez terminal:*
- S - Całkowite zatrzymanie
- F - Jazda do przodu
- B - Jazda do tyłu
- L - Skręt w lewo
- R - Skręt w prawo
- Z - Jazda zygzakiem
- T - Obrót o 180°
- I - Aktywacja/deaktywacja programu śledzenia linii
