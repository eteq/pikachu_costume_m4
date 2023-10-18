#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![allow(unused_imports)]
#![allow(dead_code)]


mod dmp_firmware;


pub(crate) fn mpu6050_setup<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) {
    let mut reg_buffer = [0; 1];
    twim.blocking_write_read(address, &[0x75], &mut reg_buffer).unwrap();
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050! {}", reg_buffer[0]); }

    // device reset
    twim.blocking_write(address, &[107, 0b10000000]).unwrap();
    Delay.delay_ms(100u32);
    // singal path reset - not clear if this is necessary but maybe recommended in register map doc?
    twim.blocking_write(address, &[104, 0b00000111]).unwrap();
    Delay.delay_ms(100u32);

    twim.blocking_write(address, &[56,  0b00000000]).unwrap(); // no interrupts
    twim.blocking_write(address, &[35,  0b00000000]).unwrap(); // Turn off regular FIFO to use DMP FIFO instead
    twim.blocking_write(address, &[28,  0b00000000]).unwrap(); // 2g full scale accelerometer
    twim.blocking_write(address, &[55,  0b10010000]).unwrap(); //logic level for int pin low, and clear int status on any read MAY NOT BE NEEDED?
    twim.blocking_write(address, &[107, 0b00000001]).unwrap(); // Clock to PLL with X axis gyroscope reference, dont sleep
    twim.blocking_write(address, &[26,  0b00000001]).unwrap(); // no external synchronization, set the DLPF to 184/188 Hz bandwidth
    twim.blocking_write(address, &[25,  0b00000100]).unwrap(); // divides the sample rate - yields 200hz I think combined with above?
	
    mpu6050_write_firmware(twim, address, true);
    // set DMP Program Start Address 0x0400
    twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_DMP_PROG_START, 0x04, 0x00]).unwrap(); 
   
    twim.blocking_write(address, &[27,  0b00011000]).unwrap(); // set gyro to +2000 Deg/sec full range
    twim.blocking_write(address, &[106, 0b11001100]).unwrap(); // turn on the fifo and reset it, and also turn on and reset the DMP (bits 7,3, respectively)
    twim.blocking_write(address, &[56,  0b00000010]).unwrap(); // now turn on the RAW_DMP_INT_EN since the DMP is now set up
  
    twim.blocking_write_read(address, &[0x75], &mut reg_buffer).unwrap();
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050 after reset/setup! {}", reg_buffer[0]); }
}

pub(crate) fn mpu6050_write_firmware<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8, verify: bool) {
    let nbanks = dmp_firmware::DMP_FIRMWARE.len() / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE + 
        if dmp_firmware::DMP_FIRMWARE.len() % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE == 0 {0} else {1};

    for b in 0..nbanks {
        let bank_size = if b == (nbanks - 1) {
            // last block
            dmp_firmware::DMP_FIRMWARE.len() - (nbanks-1) * dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        } else {
            dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        };

        // set the bank
        twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, b as u8]).unwrap();

        // now write the bank one chunk at a time
        for i in (0..bank_size).step_by(dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) {
            let chunk_size = if (i + dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) > bank_size {
                    bank_size - i
                } else {
                    dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE
                };
            let mut write_buffer = [dmp_firmware::MPU6050_REGADDR_MEM_R_W; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE+1];
            for j in 0..chunk_size {
                write_buffer[j + 1] = dmp_firmware::DMP_FIRMWARE[i + j + b*dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE];
            }
            // set the start address then write
            twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, i as u8]).unwrap();
            twim.blocking_write(address, &write_buffer[0..(chunk_size+1)]).unwrap();
        }

    }

    if verify {
        let mut verify_buffer = [0; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE];
        let mut i = 0;
        let mut last_bank = 255;
        while i < dmp_firmware::DMP_FIRMWARE.len() {
            let bank = (i / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8;
            if bank != last_bank {
                //set to new bank 
                twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, bank]).unwrap();
            }

            twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, (i % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8]).unwrap();
            twim.blocking_write_read(address, &[dmp_firmware::MPU6050_REGADDR_MEM_R_W],  
                                    &mut verify_buffer).unwrap();

            for j in 0..verify_buffer.len() {
                if (i+j) < dmp_firmware::DMP_FIRMWARE.len() {
                    if dmp_firmware::DMP_FIRMWARE[i+j] != verify_buffer[j] {
                        panic!("verification of firmware failed at {}! {} vs {}", i + j, dmp_firmware::DMP_FIRMWARE[i+j], verify_buffer[j]);
                    }
                }
            }
            i += verify_buffer.len();
            last_bank = bank;
        }
    }
}

pub(crate) fn mpu6050_reset_fifo<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) {
    let mut reg_buffer = [0; 1];
    // first read the USER_CTRL register
    twim.blocking_write_read(address, &[106], &mut reg_buffer).unwrap();
    twim.blocking_write(address, &[106, reg_buffer[0] | 0b00000100]).unwrap();  // write back the same thing but with the fifo reset bit set
}

pub(crate) async fn mpu6050_get_fifo_count<T: twim::Instance>(twim: &mut twim::Twim<'_, T>, address: u8) -> u16 {
    let mut rd_buffer = [0; 2];
    // first read the USER_CTRL register
    twim.write_read(address, &[114], &mut rd_buffer).await.unwrap();
    ((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16)
}

// entirely for testing that it actually produces a good fifo output
pub(crate) fn mpu6050_test_panic<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8, basic_fifo: bool) {
    
    if basic_fifo {
        twim.blocking_write(address, &[35,  0b01111000]).unwrap(); //regular fifo on for accel/gyro
    }

    mpu6050_reset_fifo(twim, address);
    let mut count = 0;
    let mut nzeros: usize = 0;
    while count == 0 {
        let mut rd_buffer = [0; 2];
        // first read the USER_CTRL register
        twim.blocking_write_read(address, &[114], &mut rd_buffer).unwrap();
        count = ((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16);
        //panic!("in 0 count");
        nzeros += 1;
    }

    if count >= 5 {
        let mut fifo = [0; 5];
        twim.blocking_write_read(address, &[116], &mut fifo).unwrap();
        panic!("reached non-zero count {}, after {} zeros. First 5: {},{},{},{},{}", count, nzeros, fifo[0], fifo[1], fifo[2], fifo[3], fifo[4]);
    } else {
        panic!("reached non-zero count {}, after {} zeros", count, nzeros);

    }
}


pub(crate) async fn mpu6050_read_latest<T: twim::Instance>(twim: &mut twim::Twim<'_, T>, address: u8) -> [u8; dmp_firmware::DMP_PACKET_SIZE] {
    mpu6050_reset_fifo(twim, address);

    let mut count = mpu6050_get_fifo_count(twim, address).await;
    while count < 28 {
        count = mpu6050_get_fifo_count(twim, address).await;
    }
    let mut fifo = [0; dmp_firmware::DMP_PACKET_SIZE];
    twim.write_read(address, &[116], &mut fifo).await.unwrap();
    fifo
}
