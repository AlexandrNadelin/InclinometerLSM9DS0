#ifndef __DEBUG_CELL_FORMAT_H
#define __DEBUG_CELL_FORMAT_H
#ifdef __cplusplus
 extern "C" {
#endif

 //---Атрибуты - набор флагов, описывающих свойства и поведение ячейки.---//
#define DEBUG_CELL_ATTR_READ           (unsigned char)0x80/*ячейка доступна для чтения */
#define DEBUG_CELL_ATTR_WRITE          (unsigned char)0x40/*ячейка доступна для записи */
#define DEBUG_CELL_ATTR_MRD_IGNORE     (unsigned char)0x20/*игнорировать чтение ячейки по команде «Читать всё»  Такой запрет бывает нужен, если к функции чтения ячейки привязан сложный алгоритм (какое-либо измерение или калибровка) с длительным временем выполнения, который нежелательно запускать во время массового чтения ячеек. */
#define DEBUG_CELL_ATTR_DEF            (unsigned char)0x10/*ячейка доступна для команды «По умолчанию» Установленный флаг Def_Att разрешает работу кнопки «По умолчанию» (см. рис. выше). */
#define DEBUG_CELL_ATTR_RANGE          (unsigned char)0x08/*ячейка имеет ограничение по диапазону значений */
#define DEBUG_CELL_ATTR_ROM            (unsigned char)0x01/*ячейка ссылается на память ROM (энергонезависимое хранение) Флаг-признак хранения содержимого ячейки в энергонезависимой памяти. Это информация для пользователя о том, что ресурс записи ячейки может быть ограниченным (ограничение ресурса перезаписи памяти Flash). Флаг ROM_Att может быть использован в различных диагностических алгоритмах программы Конфибут для запрета записи такой ячейки. */
//-----------------------------------------------------------------------//

//---Формат ячейки---//
//Формат содержит информацию о типе и размере данных, хранимых в ячейке. Под формат отводится 1 байт (см. пакет-ответ). Формат показывает, каким образом программе Конфибут следует интерпретировать читаемые/записываемые байт-данные.
#define DEBUG_CELL_FORMAT_RESERVE             (unsigned char)0x00/*Резерв*/

//0b0000ssss - Целое беззнаковое число; s - размер ячейки
#define DEBUG_CELL_FORMAT_UINT8               (unsigned char)0x01/**/
#define DEBUG_CELL_FORMAT_UINT16              (unsigned char)0x02/**/
#define DEBUG_CELL_FORMAT_UINT24              (unsigned char)0x03/**/
#define DEBUG_CELL_FORMAT_UINT32              (unsigned char)0x04/**/
#define DEBUG_CELL_FORMAT_UINT40              (unsigned char)0x05/**/
#define DEBUG_CELL_FORMAT_UINT48              (unsigned char)0x06/**/
#define DEBUG_CELL_FORMAT_UINT56              (unsigned char)0x07/**/
#define DEBUG_CELL_FORMAT_UINT64              (unsigned char)0x08/**/

#define DEBUG_CELL_FORMAT_Float               (unsigned char)0x09/*Число с плавающей запятой одинарной точности */
#define DEBUG_CELL_FORMAT_Double              (unsigned char)0x0A/*Число с плавающей запятой двойной точности */
#define DEBUG_CELL_FORMAT_ByteStream          (unsigned char)0x0B/*RAW-поток  */
#define DEBUG_CELL_FORMAT_StringStream        (unsigned char)0x0C/*Текстовый поток  */
 //0x0D-0x20 резерв
#define DEBUG_CELL_FORMAT_FixByteArray_len_1  (unsigned char)0x21/*1..31*//*Массив байт фиксированной длинны x - 0x20 - длинна массива  */
#define DEBUG_CELL_FORMAT_FixByteArray_len_31 (unsigned char)0x3F/*Массив байт фиксированной длинны x - 0x20 - длинна массива */
 //0x40 - резерв
#define DEBUG_CELL_FORMAT_FixStringArray_len_1  (unsigned char)0x41/*1..31*//*Массив байт фиксированной длинны x - 0x40 - длинна массива  */
#define DEBUG_CELL_FORMAT_FixStringArray_len_31 (unsigned char)0x5F/*Массив байт фиксированной длинны x - 0x40 - длинна массива */
//0x60 - резерв

 //0b0110ssss - Целое знаковое число; s - размер ячейки
#define DEBUG_CELL_FORMAT_INT8                (unsigned char)0x61/**/
#define DEBUG_CELL_FORMAT_INT16               (unsigned char)0x62/**/
#define DEBUG_CELL_FORMAT_INT24               (unsigned char)0x63/**/
#define DEBUG_CELL_FORMAT_INT32               (unsigned char)0x64/**/
#define DEBUG_CELL_FORMAT_INT40               (unsigned char)0x65/**/
#define DEBUG_CELL_FORMAT_INT48               (unsigned char)0x66/**/
#define DEBUG_CELL_FORMAT_INT56               (unsigned char)0x67/**/
#define DEBUG_CELL_FORMAT_INT64               (unsigned char)0x68/**/
//0x69-0x80 резерв

//b100nnnkk
#define DEBUG_CELL_FORMAT_FixByteArray256_LOW    (unsigned char)0x81/*256…57344*//*Массив байт фиксированной длины; размер ячейки (байт) считается по формуле: (k+4)*2^(n+6).Пример: 0x88 => n=2, k=0 => (0+4)*2^(2+6) = 4*2^8 = 1024 (байт) */
#define DEBUG_CELL_FORMAT_FixByteArray256_HIGH   (unsigned char)0x9F/*Массив байт фиксированной длины; размер ячейки (байт) считается по формуле: (k+4)*2^(n+6).Пример: 0x88 => n=2, k=0 => (0+4)*2^(2+6) = 4*2^8 = 1024 (байт) */

//0b101sssss
#define DEBUG_CELL_FORMAT_VarByteArray_len_0     (unsigned char)0xA0/*0..31*//*Массив байт переменной длины;s - размер массива (возвращается в ответ на команду чтения); по команде запроса формата отладочной ячейки возвращается значение 0xA0 */
#define DEBUG_CELL_FORMAT_VarByteArray_len_31    (unsigned char)0xBF/*Массив байт переменной длины;s - размер массива (возвращается в ответ на команду чтения); по команде запроса формата отладочной ячейки возвращается значение 0xA0 */

 //0b11sssss
#define DEBUG_CELL_FORMAT_VarStringArray_len_0   (unsigned char)0xC0/*0..31*//*Текстовая строка переменной длины;s - длина строки (возвращается в ответ на команду чтения); по команде запроса формата отладочной ячейки возвращается значение 0xC0 */
#define DEBUG_CELL_FORMAT_VarStringArray_len_31  (unsigned char)0xDF/*Текстовая строка переменной длины;s - длина строки (возвращается в ответ на команду чтения); по команде запроса формата отладочной ячейки возвращается значение 0xC0 */

//0b111sssss
#define DEBUG_CELL_FORMAT_FixByteArray8_LOW      (unsigned char)0xE1/*8..248 *//*Массив байт фиксированной длины;размер ячейки кратен 8: Len = 8 * s (байт) */
#define DEBUG_CELL_FORMAT_FixByteArray8_HIGH     (unsigned char)0xFF/*Массив байт фиксированной длины;размер ячейки кратен 8: Len = 8 * s (байт) */
//----------------------------------------------------------------------------------------------------------------------------------------------//

//---Команды режима отладки - второй байт в запросе, первый у всех UP_CMD_DEBUG---//
 //---Запросы---//
#define DEBUG_READ_CELL_LIST                   (unsigned char)0x85/*Получить список номеров ячеек*/
#define DEBUG_READ_CELL_TYPE                   (unsigned char)0x80/*Получить тип конкретной ячейки*/
#define DEBUG_READ_CELL                        (unsigned char)0x81/*Читать содержимое отладочной ячейки*/
#define DEBUG_EXTREAD_CELL                     (unsigned char)0x91/*Читать содержимое больших отладочных ячеек, которые превышают размер полезной нагрузки пакета универсального интерфейса*/
#define DEBUG_WRITE_CELL                       (unsigned char)0x82/*Запись отладочной ячейки*/
#define DEBUG_EXTWRITE_CELL                    (unsigned char)0x92/*Запись большой отладочной ячейки*/
//Файл описания отладочных ячеек
#define DEBUG_GET_CELLS_DESCRIPT_FILE          (unsigned char)0xA1/*Получить размер файла описания*/
#define DEBUG_READ_CELLS_DESCRIPT_FILE         (unsigned char)0xA2/*Чтения файла описания*/
//-------------------------------------------------------------------------------------------------//

//---Ответы---//
#define DEBUG_ANS_OK                          (unsigned char)0xF0
//------------//
//----------------------------//

typedef struct{
	unsigned short number;   /*Номер*/
	unsigned char  attribute;/*Атрибуты*/
	unsigned char  format;   /*Формат*/
	char*          name;     /*Имя*/
	unsigned char* data;     /*Данные (зависят от формата)*/
}DebugCell;

unsigned short GetDebugCellLenght(unsigned char  format);


#ifdef __cplusplus
 }
#endif
#endif


