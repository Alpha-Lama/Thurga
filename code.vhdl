library ieee;
use ieee.std_logic_1164.all;

use ieee.numeric_std.all;

entity Haptic_IC is
    port (
        clk   : in std_logic;
        reset : in std_logic;
        -- add more ports here as needed
		  position_sensor  : in  std_logic_vector(15 downto 0);
		  -- input from position sensor (e.g., ADC)
        force_sensor     : in  std_logic_vector(15 downto 0);
		  -- input from force/torque sensor
        remote_cmd       : in  std_logic_vector(7 downto 0); 
		  -- remote operation command data
        actuate_pwm      : out std_logic_vector(7 downto 0);  
		  -- output to haptic actuator (PWM or DAC)
        feedback_data    : out std_logic_vector(15 downto 0);
		  -- feedback signal to remote operator
		  -- SPI interface
        spi_mosi         : in  std_logic;
        spi_miso         : out std_logic;
        spi_sck          : in  std_logic;
        spi_cs           : in  std_logic;

        -- UART interface
        uart_rx          : in  std_logic;
        uart_tx          : out std_logic;

        -- Ethernet (RMII example)
        eth_tx_clk       : out std_logic;
        eth_tx_en        : out std_logic;
        eth_txd          : out std_logic_vector(1 downto 0);
        eth_rx_clk       : in  std_logic;
        eth_rx_dv        : in  std_logic;
        eth_rxd          : in  std_logic_vector(1 downto 0)
        -- add more ports as needed
        -- add more ports here as needed (e.g., SPI/UART for communication, status LEDs, etc.)
   
    );
end entity Haptic_IC;

architecture rtl of Haptic_IC is
    -- signal and component declarations go here
	 -- Internal signal for processed position
    signal proc_position : unsigned(15 downto 0);
    -- Internal signal for processed force
    signal proc_force    : unsigned(15 downto 0);
    -- Signal for local calculated feedback force
    signal feedback_force : unsigned(31 downto 0);
    -- PWM signal for actuator
    signal pwm_val       : unsigned(7 downto 0);
    -- Latency reduction buffer
    signal cmd_buffer    : std_logic_vector(7 downto 0);

    -- Example: Simple PID Controller signals
    signal error         : signed(15 downto 0);
    signal integral      : signed(15 downto 0);
    signal derivative    : signed(15 downto 0);
    signal prev_error    : signed(15 downto 0);

    -- Constants for PID (tune these for your system)
    constant Kp : integer := 2;
    constant Ki : integer := 1;
    constant Kd : integer := 1;
	 
	 
	  -- Internal signals for SPI
    signal spi_rx_data   : std_logic_vector(7 downto 0);
    signal spi_tx_data   : std_logic_vector(7 downto 0);
    signal spi_data_ready: std_logic;

    -- Internal signals for UART
    signal uart_rx_data   : std_logic_vector(7 downto 0);
    signal uart_tx_data   : std_logic_vector(7 downto 0);
    signal uart_data_ready: std_logic;

    -- Internal signals for Ethernet (RMII example)
    signal eth_rx_data   : std_logic_vector(7 downto 0);
    signal eth_tx_data   : std_logic_vector(7 downto 0);
    signal eth_data_ready: std_logic;
	 
	 -- Internal signals
    signal position      : signed(15 downto 0);
    signal prev_position : signed(15 downto 0);
    signal velocity      : signed(15 downto 0);
    signal impedance_force : signed(31 downto 0);

    -- Virtual environment parameters (tune as needed)
    constant K_spring : integer := 150; -- Spring constant
    constant B_damp   : integer := 30;  -- Damping constant


begin
    -- your design logic goes here
	 
	 
	  -- Position and force processing (simulate ADC to unsigned conversion)
    proc_position <= unsigned(position_sensor);
    proc_force    <= unsigned(force_sensor);

    -- Remote command buffer (for latency reduction, simple pass-through here)
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                cmd_buffer <= (others => '0');
            else
                cmd_buffer <= remote_cmd;
            end if;
        end if;
    end process;

    -- Simple PID controller for precision force feedback
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                error      <= (others => '0');
                integral   <= (others => '0');
                derivative <= (others => '0');
                prev_error <= (others => '0');
            else
                -- Desired position could be sent via remote_cmd, for now use as example
                error      <= signed(std_logic_vector(resize(unsigned(remote_cmd & "00000000"), 16))) - signed(proc_position);
                integral   <= integral + error;
                derivative <= error - prev_error;
                prev_error <= error;
					 
            end if;
        end if;
    end process;
	  -- your design logic goes here

    -- =========================
    -- SPI MODULE (placeholder)
    -- =========================
    -- Example: instantiate or code your SPI slave here and map the ports
    -- spi_inst: spi_slave
    --     port map (
    --         clk      => clk,
    --         reset    => reset,
    --         mosi     => spi_mosi,
    --         miso     => spi_miso,
    --         sck      => spi_sck,
    --         cs       => spi_cs,
    --         rx_data  => spi_rx_data,
    --         tx_data  => spi_tx_data,
    --         data_ready => spi_data_ready
    --     );

    -- =========================
    -- UART MODULE (placeholder)
    -- =========================
    -- uart_inst: uart_module
    --     port map (
    --         clk      => clk,
    --         reset    => reset,
    --         rx       => uart_rx,
    --         tx       => uart_tx,
    --         rx_data  => uart_rx_data,
    --         tx_data  => uart_tx_data,
    --         data_ready => uart_data_ready
    --     );

    -- =========================
    -- ETHERNET RMII MODULE (placeholder)
    -- =========================
    -- eth_inst: eth_rmii_mac
    --     port map (
    --         clk          => clk,
    --         reset        => reset,
    --         tx_clk       => eth_tx_clk,
    --         tx_en        => eth_tx_en,
    --         txd          => eth_txd,
    --         rx_clk       => eth_rx_clk,
    --         rx_dv        => eth_rx_dv,
    --         rxd          => eth_rxd,
    --         rx_data      => eth_rx_data,
    --         tx_data      => eth_tx_data,
    --         data_ready   => eth_data_ready
    --     );

    -- ========== Example usage ==========
    -- You can route remote commands through any of these interfaces
    -- For example, select remote command source
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                -- initialization
            else
                -- Priority: SPI > UART > Ethernet > local port
                if spi_data_ready = '1' then
                    -- Process SPI command
                elsif uart_data_ready = '1' then
                    -- Process UART command
                elsif eth_data_ready = '1' then
                    -- Process Ethernet command
                else
                    -- Use local remote_cmd port
                end if;
            end if;
        end if;
    end process;

    -- Calculate force feedback (simple PID formula)
    feedback_force <= unsigned(
        resize(
            Kp * error + Ki * integral + Kd * derivative,
            32
				
        )
    );

    -- PWM output for actuator (truncate to 8 bits)
    pwm_val <= feedback_force(15 downto 8);
    --actuate_pwm <= std_logic_vector(pwm_val);

    -- Feedback data to remote operator (could be force, position, or combined)
    --feedback_data <= std_logic_vector(feedback_force);
	 
	 
	  -- Convert position sensor input to signed
    position <= signed(position_sensor);

    -- Estimate velocity (simple discrete derivative)
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                prev_position <= (others => '0');
            else
                prev_position <= position;
            end if;
        end if;
    end process;

    velocity <= position - prev_position;

    -- Impedance rendering: F = -Kx - Bv
    impedance_force <= (-K_spring * position) - (B_damp * velocity);

    -- Convert force to actuator PWM (truncate and clip as needed)
    actuate_pwm <= std_logic_vector(impedance_force(15 downto 8)); -- Example mapping

    -- Feedback to remote (could be position, force, or both)
    --feedback_data <= std_logic_vector(impedance_force);
	 feedback_data <= std_logic_vector(impedance_force(31 downto 16));

    -- Add more sophisticated logic, communication interfaces, safety checks, etc., as needed
	 

end architecture rtl; 
