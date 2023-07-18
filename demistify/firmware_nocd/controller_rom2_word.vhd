library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity controller_rom2 is
generic	(
	ADDR_WIDTH : integer := 8; -- ROM's address width (words, not bytes)
	COL_WIDTH  : integer := 8;  -- Column width (8bit -> byte)
	NB_COL     : integer := 4  -- Number of columns in memory
	);
port (
	clk : in std_logic;
	reset_n : in std_logic := '1';
	addr : in std_logic_vector(ADDR_WIDTH-1 downto 0);
	q : out std_logic_vector(31 downto 0);
	-- Allow writes - defaults supplied to simplify projects that don't need to write.
	d : in std_logic_vector(31 downto 0) := X"00000000";
	we : in std_logic := '0';
	bytesel : in std_logic_vector(3 downto 0) := "1111"
);
end entity;

architecture arch of controller_rom2 is

-- type word_t is std_logic_vector(31 downto 0);
type ram_type is array (0 to 2 ** ADDR_WIDTH - 1) of std_logic_vector(NB_COL * COL_WIDTH - 1 downto 0);

signal ram : ram_type :=
(

     0 => x"38780042",
     1 => x"6cc6c26a",
     2 => x"00600038",
     3 => x"00006000",
     4 => x"5e0e0060",
     5 => x"0e5d5c5b",
     6 => x"c24c711e",
     7 => x"4dbff2ef",
     8 => x"1ec04bc0",
     9 => x"c702ab74",
    10 => x"48a6c487",
    11 => x"87c578c0",
    12 => x"c148a6c4",
    13 => x"1e66c478",
    14 => x"dfee4973",
    15 => x"c086c887",
    16 => x"efef49e0",
    17 => x"4aa5c487",
    18 => x"f0f0496a",
    19 => x"87c6f187",
    20 => x"83c185cb",
    21 => x"04abb7c8",
    22 => x"2687c7ff",
    23 => x"4c264d26",
    24 => x"4f264b26",
    25 => x"c24a711e",
    26 => x"c25af6ef",
    27 => x"c748f6ef",
    28 => x"ddfe4978",
    29 => x"1e4f2687",
    30 => x"4a711e73",
    31 => x"03aab7c0",
    32 => x"cfc287d3",
    33 => x"c405bffe",
    34 => x"c24bc187",
    35 => x"c24bc087",
    36 => x"c45bc2d0",
    37 => x"c2d0c287",
    38 => x"fecfc25a",
    39 => x"9ac14abf",
    40 => x"49a2c0c1",
    41 => x"fc87e8ec",
    42 => x"fecfc248",
    43 => x"effe78bf",
    44 => x"4a711e87",
    45 => x"721e66c4",
    46 => x"87f5e949",
    47 => x"1e4f2626",
    48 => x"bffecfc2",
    49 => x"87c8e649",
    50 => x"48eaefc2",
    51 => x"c278bfe8",
    52 => x"ec48e6ef",
    53 => x"efc278bf",
    54 => x"494abfea",
    55 => x"ca99ffcf",
    56 => x"48722ab7",
    57 => x"efc2b071",
    58 => x"4f2658f2",
    59 => x"5c5b5e0e",
    60 => x"4b710e5d",
    61 => x"c287c8ff",
    62 => x"c048e5ef",
    63 => x"e5497350",
    64 => x"497087f5",
    65 => x"cb9cc24c",
    66 => x"f9cb49ee",
    67 => x"4d497087",
    68 => x"97e5efc2",
    69 => x"e2c105bf",
    70 => x"4966d087",
    71 => x"bfeeefc2",
    72 => x"87d60599",
    73 => x"c24966d4",
    74 => x"99bfe6ef",
    75 => x"7387cb05",
    76 => x"87c3e549",
    77 => x"c1029870",
    78 => x"4cc187c1",
    79 => x"7587c0fe",
    80 => x"87cecb49",
    81 => x"c6029870",
    82 => x"e5efc287",
    83 => x"c250c148",
    84 => x"bf97e5ef",
    85 => x"87e3c005",
    86 => x"bfeeefc2",
    87 => x"9966d049",
    88 => x"87d6ff05",
    89 => x"bfe6efc2",
    90 => x"9966d449",
    91 => x"87caff05",
    92 => x"c2e44973",
    93 => x"05987087",
    94 => x"7487fffe",
    95 => x"87dcfb48",
    96 => x"5c5b5e0e",
    97 => x"86f40e5d",
    98 => x"ec4c4dc0",
    99 => x"a6c47ebf",
   100 => x"f2efc248",
   101 => x"1ec178bf",
   102 => x"49c71ec0",
   103 => x"c887cdfd",
   104 => x"02987086",
   105 => x"49ff87cd",
   106 => x"c187ccfb",
   107 => x"c6e349da",
   108 => x"c24dc187",
   109 => x"bf97e5ef",
   110 => x"d587c302",
   111 => x"efc287de",
   112 => x"c24bbfea",
   113 => x"05bffecf",
   114 => x"c487dac1",
   115 => x"c0c248a6",
   116 => x"c278c0c0",
   117 => x"6e7eeedd",
   118 => x"6e49bf97",
   119 => x"7080c148",
   120 => x"d2e2717e",
   121 => x"02987087",
   122 => x"66c487c3",
   123 => x"4866c4b3",
   124 => x"c828b7c1",
   125 => x"987058a6",
   126 => x"87dbff05",
   127 => x"e149fdc3",
   128 => x"fac387f5",
   129 => x"87efe149",
   130 => x"ffcf4973",
   131 => x"c01e7199",
   132 => x"87ddfa49",
   133 => x"b7ca4973",
   134 => x"c11e7129",
   135 => x"87d1fa49",
   136 => x"ffc586c8",
   137 => x"eeefc287",
   138 => x"029b4bbf",
   139 => x"cfc287dd",
   140 => x"c749bffa",
   141 => x"987087dc",
   142 => x"c087c405",
   143 => x"c287d24b",
   144 => x"c1c749e0",
   145 => x"fecfc287",
   146 => x"c287c658",
   147 => x"c048facf",
   148 => x"c2497378",
   149 => x"87cd0599",
   150 => x"e049ebc3",
   151 => x"497087d9",
   152 => x"c20299c2",
   153 => x"734cfb87",
   154 => x"0599c149",
   155 => x"f4c387cd",
   156 => x"87c3e049",
   157 => x"99c24970",
   158 => x"fa87c202",
   159 => x"c849734c",
   160 => x"87ce0599",
   161 => x"ff49f5c3",
   162 => x"7087ecdf",
   163 => x"0299c249",
   164 => x"efc287d5",
   165 => x"ca02bff6",
   166 => x"88c14887",
   167 => x"58faefc2",
   168 => x"ff87c2c0",
   169 => x"734dc14c",
   170 => x"0599c449",
   171 => x"f2c387ce",
   172 => x"c2dfff49",
   173 => x"c2497087",
   174 => x"87dc0299",
   175 => x"bff6efc2",
   176 => x"b7c7487e",
   177 => x"cbc003a8",
   178 => x"c1486e87",
   179 => x"faefc280",
   180 => x"87c2c058",
   181 => x"4dc14cfe",
   182 => x"ff49fdc3",
   183 => x"7087d8de",
   184 => x"0299c249",
   185 => x"c287d5c0",
   186 => x"02bff6ef",
   187 => x"c287c9c0",
   188 => x"c048f6ef",
   189 => x"87c2c078",
   190 => x"4dc14cfd",
   191 => x"ff49fac3",
   192 => x"7087f4dd",
   193 => x"0299c249",
   194 => x"c287d9c0",
   195 => x"48bff6ef",
   196 => x"03a8b7c7",
   197 => x"c287c9c0",
   198 => x"c748f6ef",
   199 => x"87c2c078",
   200 => x"4dc14cfc",
   201 => x"03acb7c0",
   202 => x"c487d1c0",
   203 => x"d8c14a66",
   204 => x"c0026a82",
   205 => x"4b6a87c6",
   206 => x"0f734974",
   207 => x"f0c31ec0",
   208 => x"49dac11e",
   209 => x"c887e5f6",
   210 => x"02987086",
   211 => x"c887e2c0",
   212 => x"efc248a6",
   213 => x"c878bff6",
   214 => x"91cb4966",
   215 => x"714866c4",
   216 => x"6e7e7080",
   217 => x"c8c002bf",
   218 => x"4bbf6e87",
   219 => x"734966c8",
   220 => x"029d750f",
   221 => x"c287c8c0",
   222 => x"49bff6ef",
   223 => x"c287d3f2",
   224 => x"02bfc2d0",
   225 => x"4987ddc0",
   226 => x"7087c7c2",
   227 => x"d3c00298",
   228 => x"f6efc287",
   229 => x"f9f149bf",
   230 => x"f349c087",
   231 => x"d0c287d9",
   232 => x"78c048c2",
   233 => x"f3f28ef4",
   234 => x"5b5e0e87",
   235 => x"1e0e5d5c",
   236 => x"efc24c71",
   237 => x"c149bff2",
   238 => x"c14da1cd",
   239 => x"7e6981d1",
   240 => x"cf029c74",
   241 => x"4ba5c487",
   242 => x"efc27b74",
   243 => x"f249bff2",
   244 => x"7b6e87d2",
   245 => x"c4059c74",
   246 => x"c24bc087",
   247 => x"734bc187",
   248 => x"87d3f249",
   249 => x"c70266d4",
   250 => x"87da4987",
   251 => x"87c24a70",
   252 => x"d0c24ac0",
   253 => x"f1265ac6",
   254 => x"000087e2",
   255 => x"00000000",
   256 => x"00000000",
   257 => x"711e0000",
   258 => x"bfc8ff4a",
   259 => x"48a17249",
   260 => x"ff1e4f26",
   261 => x"fe89bfc8",
   262 => x"c0c0c0c0",
   263 => x"c401a9c0",
   264 => x"c24ac087",
   265 => x"724ac187",
   266 => x"0e4f2648",
   267 => x"5d5c5b5e",
   268 => x"ff4b710e",
   269 => x"66d04cd4",
   270 => x"d678c048",
   271 => x"f6daff49",
   272 => x"7cffc387",
   273 => x"ffc3496c",
   274 => x"494d7199",
   275 => x"c199f0c3",
   276 => x"cb05a9e0",
   277 => x"7cffc387",
   278 => x"98c3486c",
   279 => x"780866d0",
   280 => x"6c7cffc3",
   281 => x"31c8494a",
   282 => x"6c7cffc3",
   283 => x"72b2714a",
   284 => x"c331c849",
   285 => x"4a6c7cff",
   286 => x"4972b271",
   287 => x"ffc331c8",
   288 => x"714a6c7c",
   289 => x"48d0ffb2",
   290 => x"7378e0c0",
   291 => x"87c2029b",
   292 => x"48757b72",
   293 => x"4c264d26",
   294 => x"4f264b26",
   295 => x"0e4f261e",
   296 => x"0e5c5b5e",
   297 => x"1e7686f8",
   298 => x"fd49a6c8",
   299 => x"86c487fd",
   300 => x"486e4b70",
   301 => x"c201a8c0",
   302 => x"4a7387f0",
   303 => x"c19af0c3",
   304 => x"c702aad0",
   305 => x"aae0c187",
   306 => x"87dec205",
   307 => x"99c84973",
   308 => x"ff87c302",
   309 => x"4c7387c6",
   310 => x"acc29cc3",
   311 => x"87c2c105",
   312 => x"c94966c4",
   313 => x"c41e7131",
   314 => x"92d44a66",
   315 => x"49faefc2",
   316 => x"d2fe8172",
   317 => x"49d887f9",
   318 => x"87fbd7ff",
   319 => x"c21ec0c8",
   320 => x"fd49eade",
   321 => x"ff87d4ef",
   322 => x"e0c048d0",
   323 => x"eadec278",
   324 => x"4a66cc1e",
   325 => x"efc292d4",
   326 => x"817249fa",
   327 => x"87ccd1fe",
   328 => x"acc186cc",
   329 => x"87c2c105",
   330 => x"c94966c4",
   331 => x"c41e7131",
   332 => x"92d44a66",
   333 => x"49faefc2",
   334 => x"d1fe8172",
   335 => x"dec287f1",
   336 => x"66c81eea",
   337 => x"c292d44a",
   338 => x"7249faef",
   339 => x"d8cffe81",
   340 => x"ff49d787",
   341 => x"c887e0d6",
   342 => x"dec21ec0",
   343 => x"edfd49ea",
   344 => x"86cc87e3",
   345 => x"c048d0ff",
   346 => x"8ef878e0",
   347 => x"0e87e7fc",
   348 => x"5d5c5b5e",
   349 => x"4d711e0e",
   350 => x"d44cd4ff",
   351 => x"c3487e66",
   352 => x"c506a8b7",
   353 => x"c148c087",
   354 => x"497587e2",
   355 => x"87f0dffe",
   356 => x"66c41e75",
   357 => x"c293d44b",
   358 => x"7383faef",
   359 => x"eccafe49",
   360 => x"6b83c887",
   361 => x"48d0ff4b",
   362 => x"dd78e1c8",
   363 => x"c349737c",
   364 => x"7c7199ff",
   365 => x"b7c84973",
   366 => x"99ffc329",
   367 => x"49737c71",
   368 => x"c329b7d0",
   369 => x"7c7199ff",
   370 => x"b7d84973",
   371 => x"c07c7129",
   372 => x"7c7c7c7c",
   373 => x"7c7c7c7c",
   374 => x"7c7c7c7c",
   375 => x"c478e0c0",
   376 => x"49dc1e66",
   377 => x"87f4d4ff",
   378 => x"487386c8",
   379 => x"87e4fa26",
   380 => x"5c5b5e0e",
   381 => x"711e0e5d",
   382 => x"4bd4ff7e",
   383 => x"f0c21e6e",
   384 => x"c9fe49ce",
   385 => x"86c487c7",
   386 => x"029d4d70",
   387 => x"c287c3c3",
   388 => x"4cbfd6f0",
   389 => x"ddfe496e",
   390 => x"d0ff87e6",
   391 => x"78c5c848",
   392 => x"c07bd6c1",
   393 => x"c17b154a",
   394 => x"b7e0c082",
   395 => x"87f504aa",
   396 => x"c448d0ff",
   397 => x"78c5c878",
   398 => x"c17bd3c1",
   399 => x"7478c47b",
   400 => x"fcc1029c",
   401 => x"eadec287",
   402 => x"4dc0c87e",
   403 => x"acb7c08c",
   404 => x"c887c603",
   405 => x"c04da4c0",
   406 => x"dbebc24c",
   407 => x"d049bf97",
   408 => x"87d20299",
   409 => x"f0c21ec0",
   410 => x"cafe49ce",
   411 => x"86c487fb",
   412 => x"c04a4970",
   413 => x"dec287ef",
   414 => x"f0c21eea",
   415 => x"cafe49ce",
   416 => x"86c487e7",
   417 => x"ff4a4970",
   418 => x"c5c848d0",
   419 => x"7bd4c178",
   420 => x"7bbf976e",
   421 => x"80c1486e",
   422 => x"8dc17e70",
   423 => x"87f0ff05",
   424 => x"c448d0ff",
   425 => x"059a7278",
   426 => x"48c087c5",
   427 => x"c187e5c0",
   428 => x"cef0c21e",
   429 => x"cfc8fe49",
   430 => x"7486c487",
   431 => x"c4fe059c",
   432 => x"48d0ff87",
   433 => x"c178c5c8",
   434 => x"7bc07bd3",
   435 => x"48c178c4",
   436 => x"48c087c2",
   437 => x"264d2626",
   438 => x"264b264c",
   439 => x"5b5e0e4f",
   440 => x"4b710e5c",
   441 => x"d80266cc",
   442 => x"f0c04c87",
   443 => x"87d8028c",
   444 => x"8ac14a74",
   445 => x"8a87d102",
   446 => x"8a87cd02",
   447 => x"d787c902",
   448 => x"fb497387",
   449 => x"87d087ea",
   450 => x"49c01e74",
   451 => x"7487e0f9",
   452 => x"f949731e",
   453 => x"86c887d9",
   454 => x"0087fcfe",
   455 => x"eaddc21e",
   456 => x"b9c149bf",
   457 => x"59eeddc2",
   458 => x"c348d4ff",
   459 => x"d0ff78ff",
   460 => x"78e1c848",
   461 => x"c148d4ff",
   462 => x"7131c478",
   463 => x"48d0ff78",
   464 => x"2678e0c0",
   465 => x"ddc21e4f",
   466 => x"f0c21ede",
   467 => x"c3fe49ce",
   468 => x"86c487fb",
   469 => x"c3029870",
   470 => x"87c0ff87",
   471 => x"35314f26",
   472 => x"205a484b",
   473 => x"46432020",
   474 => x"00000047",
   475 => x"9f1a0000",
   476 => x"14111258",
   477 => x"231c1b1d",
   478 => x"595aa74a",
   479 => x"f2f59491",
   480 => x"f2f5f4eb",
  others => ( x"00000000")
);

-- Xilinx Vivado attributes
attribute ram_style: string;
attribute ram_style of ram: signal is "block";

signal q_local : std_logic_vector((NB_COL * COL_WIDTH)-1 downto 0);

signal wea : std_logic_vector(NB_COL - 1 downto 0);

begin

	output:
	for i in 0 to NB_COL - 1 generate
		q((i + 1) * COL_WIDTH - 1 downto i * COL_WIDTH) <= q_local((i+1) * COL_WIDTH - 1 downto i * COL_WIDTH);
	end generate;
    
    -- Generate write enable signals
    -- The Block ram generator doesn't like it when the compare is done in the if statement it self.
    wea <= bytesel when we = '1' else (others => '0');

    process(clk)
    begin
        if rising_edge(clk) then
            q_local <= ram(to_integer(unsigned(addr)));
            for i in 0 to NB_COL - 1 loop
                if (wea(NB_COL-i-1) = '1') then
                    ram(to_integer(unsigned(addr)))((i + 1) * COL_WIDTH - 1 downto i * COL_WIDTH) <= d((i + 1) * COL_WIDTH - 1 downto i * COL_WIDTH);
                end if;
            end loop;
        end if;
    end process;

end arch;