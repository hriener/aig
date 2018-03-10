/* aig: an AIG library
 * Copyright (C) 2018  EPFL
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <lorina/aig.hpp>
#include <aig/aig.hpp>

class reader : public lorina::aig_reader
{
public:
  void on_and( unsigned index, unsigned left_lit, unsigned right_lit ) const override
  {
    ++count_and;
  }

  void on_latch( unsigned index, latch_init_value init_value ) const override
  {
    ++count_latch;
  }

  mutable unsigned count_and = 0u;
  mutable unsigned count_latch = 0u;
}; /* aig_reader */

int main( int argc, char *argv[] )
{
  if ( argc != 2 )
  {
    std::cout << "usage: " << argv[0] << ' ' << "<aig-file>" << std::endl;
    return -1;
  }

  reader r;
  const auto result = lorina::read_aig( argv[1], r );
  if ( result == lorina::return_code::success )
  {
    std::cout << "parsing successful" << std::endl;
    std::cout << "#ands = " << r.count_and << std::endl;
    std::cout << "#latches = " << r.count_latch << std::endl;
  }
  else
  {
    std::cout << "parsing failed" << std::endl;
  }
  
  return 0;
}
