#!/usr/bin/env python
# coding: utf-8

#後処理2.951

import sys
import os
import time     
import collections
import machine

rtc = machine.RTC()

SD_HOME = '/sd/'
FLASH_HOME = '/flash/'

sys.path.append(FLASH_HOME)

LineData = collections.namedtuple('LineData', ['date', 'lst'])

#定数
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 1
SERIAL_WAIT_CHAR_SIZE = 1024
RECREATE_LIMIT_SEC = 5
LINEAR_BUF_LIMIT = 2048
ERROR_LIMIT_SEC = 5

BL_RESET_TIME = 600
BL_P_P_VALUE = 20
BL_UPPER_LIMIT = 900
BL_LOWER_LIMIT = 600
RETURN_TIME_1 = 20
RETURN_TIME_2 = 20
RETURN_HOLD = 4
DELAY_TIME_R = 1
DELAY_TIME_T = 2
DELAY_COUNT_EX = 10
WAIT_TIME = 20
MA_INTERVAL = 4
BIN_CHG_DD = 2
EDGE_CENTER_DIF_GAP = 10
DELTA_DIF_MIN_VALUE = -2
DELTA_DIF_MAX_VALUE = 3
DELTA_DIF_R_MAX_VALUE  = 1.2
DIF_MIN_EX = 3
RAW_MAX_VALUE = 900
RAW_MIN_VALUE = 300
SKN_DIF_VALUE = 200
SKN_JDG_FRQ = 120
SKN_PERIOD = 3600
SKN_RATIO = 95
BROKEN_VALUE = 3600
DISCONNECT_VALUE = 10

#リスト
Jdg_list = []
Return_list_1 = []
Return_list_2 = []
Delay_list_1 = []
Delay_list_2 = []
BL_list = []
Raw_rev_list = []
Day_list = []
Dif_list = []
Dd_min_ex_flag_list = []
SKN_list = []

SKN_state = []
Broken_state = []
Disconnect_state = []

state_wait = []

#GPIO
from machine import Pin

level1_pin = Pin(25, Pin.IN, Pin.PULL_UP)
level3_pin = Pin(26, Pin.IN, Pin.PULL_UP)

LED_pin = Pin(27, Pin.OUT)
NC_pin = Pin(19, Pin.OUT)

#power switch check
SW_check_pin = Pin(33, Pin.IN, Pin.PULL_UP)
Cap_reset_pin = Pin(32, Pin.OUT)

#reboot
from machine import WDT
wdt = WDT(timeout=5000)

Reset_cause = machine.reset_cause()
# print('reset_cause =',Reset_cause)

# threshold.setting
THRESHOLD = [10,10,10,10,10,20,10,10,10,10,10,10]

#sensor_setting.setting
i2caddress = '5A'
currentList = [52,46,39,32,24,15,57,53,48,43,37,32]
tmpList = [6,6,6,6,6,6,6,6,6,6,6,6]


def getLogPath( prefix, date ):
	try:
		dayStr = date.split(' ')[0].replace('-','')
		dat = dayStr[2:]
		return SD_HOME + prefix + dat + '.csv'
	except:
		return SD_HOME + 'error.csv'

def shell_print( message ):
	dt = rtc.datetime()
	dp = int(dt[7]/1000)
	nowDate =  "{:02d}:{:02d}:{:02d}.{:03d}".format(dt[4], dt[5], dt[6], dp)
	print(nowDate,message)

def result_sd_write( prefix , nowDate , List):
	result = [nowDate]
	result.extend(List)
	with open( getLogPath( prefix, nowDate ), 'a' ) as f:
		print(*result, sep=",", file=f)

def result_flash_write( prefix , nowDate , List):
	result = [nowDate]
	result.extend(List)
	with open( FLASH_HOME + prefix + '.csv', 'w' ) as f:
		print(*result, sep=",", file=f)


class Application:
	__ReCreateFlg = False
	@staticmethod
	def SetReCreate():
		Application.__ReCreateFlg = True
		
	def __init__(self):
		self.initialize()
		
	def __del__(self):
		self.close()
		
	def initialize(self):
		self._dataClass = DataClass()
		self._loggerContainer = LoggerContainer(self._dataClass)
		self._loggerRunner = LoggerRunner( self._dataClass , self._loggerContainer )
		
	def close(self):
		self._loggerRunner.close()
		self._loggerContainer.close()
		self._dataClass.close()
		
	def run(self):
		try:
			while True: 
				self._loggerRunner._loop()
				if Application.__ReCreateFlg:
					Application.__ReCreateFlg = False
					self.close()
					self.initialize()
		except KeyboardInterrupt:
			self._loggerRunner.close()


class DataClass:
	def __init__(self):
		self._recvErrorChecker = 0
		self._sendErrorChecker = 0
		self._recreateTimer = 0
		self._setComplete = False
		self._serial = None
		self._serialInit()
		self._callBuf = []
		self._callSet = []
		self._buf = {} 
		self._connected = True
		
	def __del__(self):
		self.close()
		
	def read_UART_block_data(self,address,register,size):
		result = []
		if address not in self._buf:
			return result
		target = self._buf[address]
		for I in range(register,register+size):
			if I in target:
				result.append(target[I])
			else:
				result.append(0)
		return result
		
	def write_UART(self,address,register,value):
		data = bytearray()
		data.extend('!W'.encode('utf-8'))
		data.append(address)
		data.append(register)
		data.append(value)
		if data in self._callSet:
			return
		self._callSet.append(data)
			
	def close(self):
		self._connected = False
		if not None is self._serial:
			self._serial.deinit()
			self._serial = None
			time.sleep_ms(200) 
		self._buf_enable = True
			
	def _serialInit(self):
		if not None is self._serial:
			self._serial.deinit()
			self._serial = None
		self._serial = machine.UART(1, baudrate=SERIAL_BAUDRATE, tx=14, rx=13, timeout=SERIAL_TIMEOUT)
		self._buf_enable = True
		time.sleep_ms(SERIAL_TIMEOUT*20)
		self._recreateTimer  = time.ticks_ms()/1000
			
	def _IsInnerError(self):
		now  = time.ticks_ms()/1000
		test1 = ERROR_LIMIT_SEC < now - self._recvErrorChecker
		test2 = ERROR_LIMIT_SEC < now - self._sendErrorChecker
		return test1 or test2
		
	def hasError(self):
		return RECREATE_LIMIT_SEC < time.ticks_ms()/1000 - self._recreateTimer
		
	def _read_from_port(self):
		linear = bytearray()
		while self._connected:
			#led check
			try:
				if self._IsInnerError():
					pass
				else:
					self._recreateTimer = time.ticks_ms()/1000
			except:
				continue
			if self._buf_enable:
				if not self._connected:
					break
					
				if self._setComplete:
					for i in range(len(self._callSet)):
						self._serial.write( self._callSet.pop(0) )
						time.sleep_ms(100)
						
					if 0 < len(self._callBuf):
						self._buf_enable = False
						self._serial.write( self._callBuf.pop(0) )
						self._sendErrorChecker = time.ticks_ms()/1000
					else:
						break
			try:
				read_data = self._serial.read(SERIAL_WAIT_CHAR_SIZE)
				if len(read_data) <= 0:
					continue
			except Exception as e:
				linear = bytearray()
				self._buf_enable = True
				break
			linear.extend(read_data)
			while self._connected:
				try:
					tmp = linear
					l = len(tmp)
					HEADER_SIZE = 5
					if HEADER_SIZE <= l:
						if ( tmp[1] == 0x57 ): 
							address  = int(tmp[2])
							register = int(tmp[3])
							size     = int(tmp[4])
							data = tmp[HEADER_SIZE:]
							if size <= len(data):
								if any( data[i] != 0 for i in range(0,size)):
									self._recvErrorChecker = time.ticks_ms()/1000
									if address not in self._buf:
										self._buf[address] = {}
									for I in range(0,size):
										self._buf[address][register+I] = data[I]
								linear = linear[HEADER_SIZE+size:]
								continue
					if 2 <= l:
						if ( tmp[1] == 0x45 ): #0x45=E
							self._buf_enable = True
							linear = linear[2:]
							continue
					break     
				except:
					linear = bytearray()
					break
			if ( LINEAR_BUF_LIMIT < len(linear) ):
				linear = bytearray()
				
	def call_UART(self,address,register,size):
		if address == 0x00:
			return
		data = bytearray()
		data.extend('!R'.encode('utf-8'))
		data.append(address)
		data.append(0x00)
		data.append(register+size)
		if data in self._callBuf:
			return
		self._callBuf.append(data)
			
	def mpr121QuickConfig( self , mpr121Address ):
		#setting cdc
		currentAddressList = [0x64,0x63,0x62,0x61,0x60,0x5F,0x65,0x66,0x67,0x68,0x69,0x6A]
		#setting cdt
		zero_list = tmpList[0::2]
		one_list = tmpList[1::2]
		timeList = [ ((o<<4)&0x70)|(z&0x07) for (o,z) in zip(one_list,zero_list) ]
		timeAddressList = [0x6C,0x6D,0x6E,0x6F,0x70,0x71]
		self.write_UART(mpr121Address,0x80,0x63)
		self.write_UART(mpr121Address,0x7B,0x00) #ATO_CFG0
		self.write_UART(mpr121Address,0x5C,0)
		self.write_UART(mpr121Address,0x5D,0x04) #FIL_CFG
		for a,b in zip(currentAddressList , currentList ):
			self.write_UART(mpr121Address,a,b)
		for a,b in zip(timeAddressList , timeList ):
			self.write_UART(mpr121Address,a,b)
		#setting
		self.write_UART(mpr121Address,0x5E,0x0C) #ELE_CFG
		self._setComplete = True


class LoggerContainer:
	def __init__(self,dataClass):
		self.close()
		
	def __del__(self):
		self.close()
		
	def close(self):
		self._logger = None
		
	def recreateLogger(self,dataClass):
		self._logger = None
		self._logger = GraphSet(dataClass)
		
	def execute(self,dataClass):
		if None is self._logger:
			return
		self._logger.graphExecute( dataClass )
		
	def updateLedState(self):
		if None is self._logger:
			return
		self._logger.updateLedState()


class LoggerRunner:
	def __init__(self,dataClass,logicalLogger):
		self._running = True
		self._loggerContainer = logicalLogger
		self._dataClass = dataClass
		
	def close(self):
		self._running = False
		
	#ログ保存ループ
	def _loop(self):
		LoopMillis = 500
		L = self._loggerContainer
		D = self._dataClass
		L.recreateLogger(D)
		last = 0
		while self._running:
			wdt.feed()
			last = time.ticks_ms()
			if not self._running:
				return
			if D.hasError():
				Application.SetReCreate()
				time.sleep_ms( 500 )
				return
			D._read_from_port()
			L.execute(D)
			L.updateLedState()
# 			mid = time.ticks_ms()
# 			shell_print(mid - last)
			now = time.ticks_ms()
			sleepMillis = LoopMillis - (now - last)
			sleepMillis = sleepMillis if sleepMillis > 0 else 0
			time.sleep_ms( sleepMillis )
      

class GraphSet:
	def __init__( self, dataClass ):
		self.__graph = Graph( dataClass=dataClass )
		
	def __del__(self):
		self.__graph = None
		
	def graphExecute( self, dataClass ):
		dt = rtc.datetime()
		dp = int(dt[7]/1000)
		nowDate =  "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}.{:03d}"\
					.format(dt[0], dt[1], dt[2], dt[4], dt[5], dt[6], dp)
		if self.__graph.CheckDateChange(nowDate):
			self.__graph.callAll(dataClass)
			self.__graph.updateAll(nowDate,dataClass)
			self.__graph.MLAll(nowDate)
			
	def updateLedState(self):
		ledState = self.__graph.getLedState()
		if ledState == 1 :
			LED_pin.on()
			time.sleep_ms(100)
			LED_pin.off()
		else:
			LED_pin.off()

class Graph:
	def __init__( self, dataClass ):
		self.__graphs = []
		self.__lastChkDate = ''
		address = int( i2caddress, 16 )
		dataClass.mpr121QuickConfig( address )
		self.__graphs.append( SensorSet( DATAaddress=address, DATAcount=12, DATAoffset=0x04 ) )
		
	def __del__(self):
		self.__graphs = None
		
	def callAll(self,dataClass):
		for graph in self.__graphs:
			graph.callDataUpdate( dataClass )
			
	def CheckDateChange(self,nowDate):
		if nowDate == self.__lastChkDate:
			return False
		self.__lastChkDate = nowDate
		return True
		
	def updateAll(self,nowDate,dataClass):
		for graph in self.__graphs:
			graph.doLogging( nowDate, dataClass )
			
	def MLAll(self,nowDate):
		for graph in self.__graphs:
			binaryList = graph.preprocessing()
			MLresult = graph.MLexecute( binaryList )
			graph.notification( nowDate, MLresult )
			
	def getLedState(self):
		state = 0
		if len(state_wait) < 3 :
			state_wait.append(0)
		for graph in self.__graphs:
			state += graph.lastDataLed
		return 1 if ( state!=0 and len(state_wait)==3 ) else 0


class SensorSet:
	def __init__(self, DATAaddress, DATAcount, DATAoffset ):
		self.__DATAaddress = DATAaddress
		self.__DATAcount  = DATAcount
		self.__DATAoffset = DATAoffset
		self.lastDataLed = -1
		self.baseMeanValue = None 
		self.needUpdateBaseValue = True
		self.Base_file_value = 0
		self.lastRawBuf = [] 
		self.Temp_file_value = 0
		self.lastRev = None
		self.Dif = []
		self.Delta_dif = []
		self.Pre_binaryList = [0,0,0,0,0,0,0,0,0,0,0,0]
		self.Rise_count = 0
		self.Return_count_1 = 0
		self.Return_count_2 = 0
		self.Delay_count_1 = 0
		self.Delay_count_2 = 0
		self.Wait_count_1 = 0
		self.Wait_count_2 = 0
		self.Delta_dif_min_cnt = 0
		self.Dd_min_ex_flag = 0
		self.SKN_count = 0
		self.Broken_count = 0
		self.Disconnect_count = 0
		self.NC0_out = 0
		
	def callDataUpdate( self, dataClass ):
		da = self.__DATAaddress
		dc2 = 2 * (self.__DATAcount)
		dc2o = dc2 + self.__DATAoffset
		dataClass.call_UART( da, 0x00, dc2o )
		
	def __readFromBuf( self, dataClass ):
		da = self.__DATAaddress
		do = self.__DATAoffset
		dc = self.__DATAcount
		dc2 = 2 * dc
		dat = dataClass.read_UART_block_data( da, do, dc2 )
		if len(dat) < dc2:
			return []
		raw_lst = []
		for i in range(0,dc):
			idx0 = i*2
			idx1 = idx0 + 1
			d0 = dat[idx0]
			d1s = dat[idx1]<<8
			val = d1s|d0
			raw_lst.append ( val )
		lst_pre = [raw_lst[5],raw_lst[4],raw_lst[3],raw_lst[2],raw_lst[1],raw_lst[0],\
				   raw_lst[6],raw_lst[7],raw_lst[8],raw_lst[9],raw_lst[10],raw_lst[11]]
# 		print(lst_pre)
		Raw_rev_list.append(lst_pre)
		if len(Raw_rev_list) > 2 :
			Raw_rev_list.pop(0)
		if len(Raw_rev_list) == 1 :
			lst_rev = lst_pre
		else:
			lst_rev = []
			for i in range(12):
				if RAW_MIN_VALUE <= Raw_rev_list[1][i] <= RAW_MAX_VALUE :
					Raw_CH = Raw_rev_list[1][i]
				else:
					Raw_CH = Raw_rev_list[0][i]
				lst_rev.append(Raw_CH)
			Raw_rev_list[1] = lst_rev
		lst_list = [lst_pre , lst_rev]
		#error_check
		Over_ch = 0
		for i in range(12):
			if lst_pre[i] > RAW_MAX_VALUE:
				Over_ch += 1
		if Over_ch > 0 :
			if self.Broken_count < BROKEN_VALUE:
				self.Broken_count += 1
		else:
			self.Broken_count = 0
		if Over_ch > 3:
			if self.Disconnect_count < DISCONNECT_VALUE:
				self.Disconnect_count += 1
		else:
			self.Disconnect_count = 0
		#save
		if self.Temp_file_value == 0:
			result_flash_write('temp0', '' , [0])
			self.Temp_file_value = 1
		else:
			result_flash_write('temp1', '' , [1])
			self.Temp_file_value = 0
		return lst_list
		
	def updateBaseMeanValueIfNeeded( self, nowDate, tmpLastRawBuf ):
		if not self.needUpdateBaseValue :
			self.processlog = [0]
			return
		if len(tmpLastRawBuf) <= 4:
			self.processlog = [0]
			return
		self.needUpdateBaseValue = False
		if not BL_list : #start?
			if Reset_cause == 1 and SW_check_pin.value() == 0 :
				pass
			else:
				if Reset_cause != 1 :
					self.processlog = [2]
				else :
					self.processlog = [3]
					Cap_reset_pin.on()
					time.sleep_ms(100)
					Cap_reset_pin.off()
				Base_file_list = []
				for i in os.listdir(FLASH_HOME[:-1]):
					if 'base' in i:
						Base_file_list.append(FLASH_HOME + i)
				if Base_file_list : #restart?
					Base_file_times = [(f, os.stat(f)[8]) for f in Base_file_list]
					sorted_files = sorted(Base_file_times, key=lambda x: x[1], reverse=True)
					Latest_base_path = sorted_files[0][0]
					Latest_base_list = []
					with open(Latest_base_path) as f:
						s = f.read()
						Latest_base_list = s.strip().split(',')
					if Latest_base_list[-1] == 'E' :
						self.baseMeanValue = [float(i) for i in Latest_base_list[1:13]]
						BL_list.append(self.baseMeanValue)
						return
					elif len(Base_file_list) == 2 :
						Second_base_path = sorted_files[1][0]
						Second_base_list = []
						with open(Second_base_path) as f:
							s = f.read()
							Second_base_list = s.strip().split(',')
						self.baseMeanValue = [float(i) for i in Second_base_list[1:13]]
						BL_list.append(self.baseMeanValue)
						return
		Cap_reset_pin.on()
		time.sleep_ms(100)
		Cap_reset_pin.off()
		l = tmpLastRawBuf
		n = []
		for i in range(12):
			n.append(round((l[-1][i]+l[-2][i]+l[-3][i])/3,1))
		if not BL_list and max(n) > 1000 :
			self.processlog = [4]
			return
		self.processlog = [1]
		BL_list.append(n)
		if len(BL_list) > 2 :
			BL_list.pop(0)
		if len(BL_list) == 2 :
			BL_rev = []
			for a in range(12):
				if BL_LOWER_LIMIT < BL_list[1][a] < BL_UPPER_LIMIT :
					BL_CH = BL_list[1][a]
				else:
					BL_CH = BL_list[0][a]
				BL_rev.append(BL_CH)
			BL_list[-1] = BL_rev
		self.baseMeanValue = BL_list[-1]
		self.baseMeanValue.append('E')
		if self.Base_file_value == 0:
			result_flash_write('base0', nowDate , self.baseMeanValue)
			self.Base_file_value = 1
		else:
			result_flash_write('base1', nowDate , self.baseMeanValue)
			self.Base_file_value = 0
			
	def doLogging( self, nowDate, dataClass ):
		if 0 == self.__DATAaddress:
			return
		try:
			datalist = self.__readFromBuf( dataClass )
			datalist_raw = datalist[0]
			datalist_rev = datalist[1]
			if len(datalist_rev) > 0:
				self.lastRawBuf.append( datalist_rev )
				if len(self.lastRawBuf) > 5 :
					self.lastRawBuf.pop(0)
				self.updateBaseMeanValueIfNeeded( nowDate, self.lastRawBuf )
				if len(self.lastRawBuf) < 5 :
					if max(datalist_raw) > 1000 or min(datalist_raw) < 100 :
						self.lastDataLed = 1
					else:
						self.lastDataLed = 0
				else:
					if max(datalist_raw) > 1000 or min(datalist_raw) < 100 or self.baseMeanValue == None :
						self.lastDataLed = 1
					else:
						self.lastDataLed = 0
				if len(self.lastRawBuf) < MA_INTERVAL:
					datalist_MA = datalist_rev
				else:
					a = self.lastRawBuf[-1*MA_INTERVAL:]
					datalist_MA = []
					for i in range(12):
						d = round((a[0][i]+a[1][i]+a[2][i]+a[3][i])/4,1)
						datalist_MA.append(d)
				self.lastRev = LineData(date=nowDate,lst=datalist_MA)
				datalist_raw.extend(self.processlog)
				result_sd_write('raw',nowDate,datalist_raw)
		except:
			pass
			
	def preprocessing(self):
		if self.baseMeanValue is None:
			return None
		if self.lastRev is None:
			return None
		diff = []
		for i in range(12):
			diff.append(self.baseMeanValue[i] - self.lastRev.lst[i]) 
		if not self.Dif:
			for i in range(12):
				self.Delta_dif.append(0)
		else:
			for i in range(12):
				self.Delta_dif[i] = diff[i] - self.Dif[i]
		self.Dif = diff
		Dif_list.append(self.Dif)
		if len(Dif_list) > 2 :
			Dif_list.pop(0)
		Delta_dif_min_list = [0,1,2,3,4,5,7,8,9,10] #6,11以外
		self.Delta_dif_min = min([self.Delta_dif[i] for i in Delta_dif_min_list])
		self.Delta_dif_min_cnt = sum([self.Delta_dif[i] < DELTA_DIF_MIN_VALUE for i in Delta_dif_min_list ])
		Delta_dif_r_min_list = [0,1,4,5,6,7,10,11] #2,3,8,9以外
		self.Delta_dif_r_min = min([self.Delta_dif[i] for i in Delta_dif_r_min_list])
		max_list = [6,7,10,11]
		self.Delta_dif_max = max([self.Delta_dif[i] for i in max_list])
		self.Delta_dif_r_max = max([self.Delta_dif[i] for i in range(12)])
		self.Delta_dif_max_ch = [i for i in range(12) if self.Delta_dif[i] == max(self.Delta_dif)]
		binaryList = []
		for i in range(12):
			if diff[i] < (THRESHOLD[i] - 3) :
				binaryList.append(0)
			elif diff[i] < THRESHOLD[i] :
				if self.Pre_binaryList[i] == 0 :
					binaryList.append(0)
				elif self.Delta_dif[i] < -1*BIN_CHG_DD :
					binaryList.append(0)
				else:
					binaryList.append(1)
			elif diff[i] < (THRESHOLD[i] + 5):
				if self.Pre_binaryList[i] == 1 :
					binaryList.append(1)
				elif self.Delta_dif[i] < BIN_CHG_DD :
					binaryList.append(0)
				else:
					binaryList.append(1)
			else:
				binaryList.append(1)
		for i in range(0, 1):#CH0
			if (binaryList[i+6]==binaryList[i+7]==0) and  diff[i] < THRESHOLD[i] - 3 :
				binaryList[i] = 0
		for i in range(1, 5):#CH1-4
			if (binaryList[i+5]==binaryList[i+6]==binaryList[i+7]==0) and  diff[i] < THRESHOLD[i] - 3 :
				binaryList[i] = 0
		for i in range(5, 6):#CH5
			if (binaryList[i+5]==binaryList[i+6]==0) and  diff[i] < THRESHOLD[i] - 3 :
				binaryList[i] = 0
		self.Pre_binaryList = binaryList
		if sum(binaryList) == 0 :
			self.Rise_count += 1
		else:
			self.Rise_count = 0
		if self.Rise_count == 0 :
			self.RawMax = []
			self.RawMin = []
		elif self.Rise_count == 1:
			self.RawMax = self.lastRawBuf[-1]
			self.RawMin = self.lastRawBuf[-1]
		else:
			Max_list = []
			for a in range(12) :
				if self.RawMax[a] > self.lastRawBuf[-1][a] :
					Max_CH = self.RawMax[a]
				else:
					Max_CH = self.lastRawBuf[-1][a]
				Max_list.append(Max_CH)
			self.RawMax = Max_list
			Min_list = []
			for a in range(12):
				if self.RawMin[a] < self.lastRawBuf[-1][a] :
					Min_CH = self.RawMin[a]
				else:
					Min_CH = self.lastRawBuf[-1][a]
				Min_list.append(Min_CH)
			self.RawMin = Min_list
		if self.Rise_count == BL_RESET_TIME:
			isAllDataZero = True
			self.Rise_count = 0
			Max_Min = []
			for i in range(12):
				dif = self.RawMax[i] - self.RawMin[i]
				Max_Min.append(dif)
			BL_Reset_cancel = max(Max_Min) > BL_P_P_VALUE
			if isAllDataZero and not BL_Reset_cancel :
				self.needUpdateBaseValue = True
		return binaryList
		
	def MLexecute(self,binaryList):
		if None is binaryList:
			return None
		bin_value = 0
		for i in range(12):
			bin_value += binaryList[i]*(2**(11-i))
		if  bin_value < 64 : #000000_
			if bin_value == 0 : #000000_000000
				T1result = 0
			elif bin_value == 32 or bin_value == 48 : #000000_100000 or 000000_110000
				T1result = 11
			elif bin_value == 40 and (self.Dif[6] - self.Dif[8]) > 0  : #000000_101000
				T1result = 11
			elif bin_value == 56 and (self.Dif[6] - self.Dif[8]) > EDGE_CENTER_DIF_GAP: #000000_111000
				T1result = 11
			elif bin_value == 1 or bin_value == 3 : #000000_000001 or 000000_000011
				T1result = 12
			elif bin_value == 5 and (self.Dif[11] - self.Dif[9]) > 0 : #000000_000101
				T1result = 12 
			elif bin_value == 7 and (self.Dif[11] - self.Dif[9]) > EDGE_CENTER_DIF_GAP: #000000_000111
				T1result = 12 
			else:
				T1result = 20
		else:
			T1result = 40
		Jdg_list.append(T1result)
		if len(Jdg_list) > 2 :
			Jdg_list.pop(0)
		if Jdg_list[0] in [20,40] and Jdg_list[-1] in [11,12] :
			if self.Delta_dif_min >= DELTA_DIF_MIN_VALUE and self.Delta_dif_max <= DELTA_DIF_MAX_VALUE :
				Jdg_list[-1] = Jdg_list[0]
			elif bin_value == 56 : #000000_111000
				if self.Delta_dif_min_cnt == 1 and self.Delta_dif[8] < DELTA_DIF_MIN_VALUE :
					Jdg_list[-1] = Jdg_list[0]
			elif bin_value == 7 : #000000_000111
				if self.Delta_dif_min_cnt == 1 and self.Delta_dif[9] < DELTA_DIF_MIN_VALUE :
					Jdg_list[-1] = Jdg_list[0]
		return [bin_value,Jdg_list[-1]]
		
	def notification(self,nowDate,MLresult):
		Level_1_list = [0]				#離床
		Level_2_list = [0, 11, 12]		#離床+端坐位
# 		Level_3_list = [0, 11, 12, 20]	#離床+端坐位+長坐位
		if MLresult is None :
			return
		else:
			if MLresult[1] in Level_1_list:
				NC1_flag = 1
			else:
				NC1_flag = 0
			if MLresult[1] in Level_2_list:
				NC2_flag = 1
			else:
				NC2_flag = 0
# 			if MLresult[1] in Level_3_list:
# 				NC3_flag = 1
# 			else:
# 				NC3_flag = 0
			if level1_pin.value() == 0:
				Level = 1
# 			elif level3_pin.value() == 0:
# 				Level = 3
			else :
				Level = 2
			#return
			Return_list_1.append(NC1_flag)
			if len(Return_list_1) > RETURN_HOLD:
				Return_list_1.pop(0)
			if len(Return_list_1) == RETURN_HOLD :
				if self.Return_count_1 < RETURN_TIME_1 :
					Return_flag_1= 1
				else:
					Return_flag_1= 0
				if sum(Return_list_1) == RETURN_HOLD :
					self.Return_count_1 = 0
				elif self.Return_count_1 < RETURN_TIME_1 :
					self.Return_count_1 += 1
			else:
				Return_flag_1= 0
			Return_list_2.append(NC2_flag)
			if len(Return_list_2) > RETURN_HOLD:
				Return_list_2.pop(0)
			if len(Return_list_2) == RETURN_HOLD :
				if self.Return_count_2 < RETURN_TIME_2 :
					Return_flag_2 = 1
				else:
					Return_flag_2 = 0
				if sum(Return_list_2) == RETURN_HOLD :
					self.Return_count_2 = 0
				elif self.Return_count_2 < RETURN_TIME_2 :
					self.Return_count_2 += 1
			else:
				Return_flag_2 = 0
			#delay
			Delay_list_1.append(NC1_flag)
			if len(Delay_list_1) >  2 :
				Delay_list_1.pop(0)
			if len(Delay_list_1) == 2 : 
				if Delay_list_1[0] == 0 and Delay_list_1[1] == 1 :
					self.Delay_count_1 = 1
				elif Delay_list_1[0] == Delay_list_1[1] == 1 :
					if self.Delay_count_1 <= max([DELAY_TIME_R,DELAY_COUNT_EX]) :
						self.Delay_count_1 += 1
				else:
					self.Delay_count_1 = 0
				if self.Delay_count_1 < DELAY_TIME_R :
					Delay_flag_1 = 1
				else:
					Delay_flag_1 = 0
			else:
				Delay_flag_1 = 1
			Delay_list_2.append(NC2_flag)
			if len(Delay_list_2) >  2 :
				Delay_list_2.pop(0)
			if len(Delay_list_2) == 2 : 
				if Delay_list_2[0] == 0 and Delay_list_2[1] == 1 :
					self.Delay_count_2 = 1
				elif Delay_list_2[0] == Delay_list_2[1] == 1 :
					if self.Delay_count_2 <= DELAY_TIME_T :
						self.Delay_count_2 += 1
				else:
					self.Delay_count_2 = 0
					
				if self.Delay_count_2 < DELAY_TIME_T :
					Delay_flag_2 = 1
				else:
					Delay_flag_2 = 0
			else:
				Delay_flag_2 = 1
			#delta_dif
#			if self.Delta_dif_min <= DELTA_DIF_MIN_VALUE :
#				Delta_dif_min_flag = 1
#			else:
#				Delta_dif_min_flag = 0
#				
#			if self.Delta_dif_max >= DELTA_DIF_MAX_VALUE :
#				Delta_dif_max_flag = 1
#			else:
#				Delta_dif_max_flag = 0
			if self.Delta_dif_r_min >= DELTA_DIF_MIN_VALUE :
				Delta_dif_min_r_flag = 1
			else:
				Delta_dif_min_r_flag = 0
			
			if self.Delta_dif_r_max >= DELTA_DIF_R_MAX_VALUE :
				Delta_dif_r_max_flag = 1
			else:
				Delta_dif_r_max_flag = 0
			#wait
			if self.Wait_count_1 < WAIT_TIME :
				self.Wait_count_1 += 1
				Wait_flag_1 = 1
			else:
				Wait_flag_1 = 0
			if self.Wait_count_2 < WAIT_TIME :
				self.Wait_count_2 += 1
				Wait_flag_2 = 1
			else:
				Wait_flag_2 = 0
			#失禁
			if max(self.Dif) >= SKN_DIF_VALUE:
				SKN_list.append(1)
			else:
				SKN_list.append(0)
			if len(SKN_list) > SKN_PERIOD :
				SKN_list.pop(0)
			if self.SKN_count < SKN_JDG_FRQ :
				self.SKN_count += 1
				Dif_max_exceeded_ratio = 0
				SKN_state.append(0)
			else:
				self.SKN_count = 0
				Dif_max_exceeded_ratio = int(round(sum(SKN_list)/SKN_PERIOD*100,0))
				if Dif_max_exceeded_ratio > SKN_RATIO :
					SKN_state.append(1)
				else:
					SKN_state.append(0)
			if len(SKN_state) > 2 :
				SKN_state.pop(0)
			if SKN_state[0] == 0 and SKN_state[-1] == 1 :
				SKN_flag = 1
			else:
				SKN_flag = 0
			#破損
			if self.Broken_count == BROKEN_VALUE :
				Broken_state.append(1)
			else:
				Broken_state.append(0)
			if len(Broken_state) > 2 :
				Broken_state.pop(0)
			if Broken_state[0] == 0 and Broken_state[-1] == 1 :
				Broken_flag = 1
			else:
				Broken_flag = 0
			#接続不良
			if self.Disconnect_count == DISCONNECT_VALUE :
				Disconnect_state.append(1)
			else:
				Disconnect_state.append(0)
			if len(Disconnect_state) > 2 :
				Disconnect_state.pop(0)
			if Disconnect_state[0] == 0 and Disconnect_state[-1] == 1 :
				Disconnect_flag = 1
			else:
				Disconnect_flag = 0
			#離床
			if Return_flag_1 == Delay_flag_1 == Delta_dif_min_r_flag == Wait_flag_1 == 0 :
				if Delta_dif_r_max_flag == 1 :
					if max([self.Dif[i] for i in self.Delta_dif_max_ch]) <= 2 :
						NC_out_r = 1
					else:
						NC_out_r = -1
				else:
					NC_out_r = 2
			elif self.Dd_min_ex_flag == 1 and Wait_flag_1 == 0 :
				if self.Delay_count_1 < DELAY_COUNT_EX :
					if max([max(Dif_list[0]),max(Dif_list[1])]) < DIF_MIN_EX :
						NC_out_r = 3
						self.Dd_min_ex_flag = 0
					else:
						NC_out_r = -2
						self.Dd_min_ex_flag = 1
				else:
					NC_out_r = -3
					self.Dd_min_ex_flag = 0
			elif(Return_flag_1 == Delay_flag_1 == Delta_dif_r_max_flag == Wait_flag_1 == 0) and sum([self.Delta_dif[i] <= DELTA_DIF_MIN_VALUE for i in [2,3,8,9]]) > 0:
					NC_out_r = -4
					self.Dd_min_ex_flag = 1
			else:
					NC_out_r = -5
					self.Dd_min_ex_flag = 0
			#端坐位
			if MLresult[1] in [11,12] and Return_flag_2 == Delay_flag_2 == Wait_flag_2 == 0 :
				NC_out_t = 1
			else:
				NC_out_t = 0
			#NC0
			if SKN_flag == 1 or Broken_flag == 1 or Disconnect_flag == 1 :
				NC0_flag = SKN_flag + Broken_flag + Disconnect_flag
			else:
				NC0_flag = 0
			if NC0_flag > 0 :
				self.NC0_out = 1
			#NC1
			if NC_out_r > 0 :
				NC1_out = 1
			else:
				NC1_out = 0
			#NC2
			if NC_out_r > 0 or NC_out_t == 1 :
				NC2_out = 1
			else:
				NC2_out = 0
			#NC_out
			if (Level == 1 and NC1_out == 1) or (Level == 2 and NC2_out == 1) :
				NC_out = 1
			else :
				NC_out = 0
			#call
			if self.NC0_out == 1 or NC_out == 1 :
				NC_pin.on()
			else:
				NC_pin.off()
			#output
			result = MLresult
			result.append(Level)
#			result.append(NC1_flag)
#			result.append(NC2_flag)
			result.append(self.Return_count_1)
			result.append(self.Return_count_2)
			result.append(self.Delay_count_1)
			result.append(self.Delay_count_2)
			result.append(self.Delta_dif_min)
			result.append(self.Delta_dif_min_cnt)
			result.append(self.Delta_dif_max)
			result.append(self.Delta_dif_r_min)
			result.append(self.Delta_dif_r_max)
			result.append(self.Wait_count_1)
			result.append(self.Wait_count_2)
			result.append(Dif_max_exceeded_ratio)
			result.append(self.Broken_count)
			result.append(self.Disconnect_count)
			result.append(NC0_flag)
			result.append(NC_out_r)
			result.append(NC_out_t)
			result.append(NC_out)
			result_sd_write('bjn',nowDate,result)
			#wait_reset
			if Level == 1 :
				if NC_out_r == 1 :
					self.Wait_count_1 = 0
				if NC_out_t == 1 :
					self.Wait_count_2 = 0
			else: #Level_2
				if NC_out == 1 :
					self.Wait_count_1 = 0
					self.Wait_count_2 = 0


if __name__ == '__main__':
	application = Application()
	application.run()
	application = None