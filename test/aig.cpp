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

TEST_CASE( "combinational", "[aig]" )
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

TEST_CASE( "sequential", "[aig]" )
{
  aig_graph aig( "aig" );

  /* cell #0 */
  auto in0 = aig.create_pi( "in0" );
  auto p0 = aig.create_pi( "p0" );
  auto q0 = aig.create_pi( "q0" );
  auto l0 = aig.create_li( "l0" );
  auto f0 = aig.create_ite( p0, in0, l0 );
  aig.create_latch( l0, f0, false );
  auto f1 = aig.create_ite( q0, l0, aig.get_constant( false ) );
  aig.create_po( f1, "out0" );

  /* cell #1 */
  auto in1 = aig.create_pi( "in1" );
  auto p1 = aig.create_pi( "p1" );
  auto q1 = aig.create_pi( "q1" );
  auto l1 = aig.create_li( "l1" );
  auto f2 = aig.create_ite( p1, in1, l1 );
  aig.create_latch( l1, f2, true );
  auto f3 = aig.create_ite( q1, l1, aig.get_constant( false ) );
  aig.create_po( f3, "out1" );

  CHECK( aig.size() == 17 );
  CHECK( aig.inputs().size() == 6 );
  CHECK( aig.latches().size() == 2 );
  CHECK( aig.outputs().size() == 2 );
}
