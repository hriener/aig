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

#include <catch.hpp>

#include <aig/aig.hpp>

using namespace aig;

TEST_CASE( "aig_graph", "[aig]" )
{
  aig_graph aig( "aig" );
  auto a = aig.create_pi( "a" );
  auto b = aig.create_pi( "b" );
  auto c = aig.create_pi( "c" );
  auto n = aig.create_and( a, aig.create_and( b, c ) );
  aig.create_po( n, "out" );

  CHECK( aig.size() == 6 );
  CHECK( aig.inputs().size() == 3 );
  CHECK( aig.latches().size() == 0 );
  CHECK( aig.outputs().size() == 1 );

}
