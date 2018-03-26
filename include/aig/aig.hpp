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

/*!
  \file aig.hpp
  \brief Implements an and-inverter graph.

  \author Heinz Riener
*/

#pragma once

#include <algorithm>
#include <cassert>
#include <list>
#include <unordered_map>
#include <vector>

namespace util
{

/******************************************************************************
 * dirty                                                                      *
 ******************************************************************************/

template<typename T>
class dirty
{
public:
  T& operator*() { return _m; }
  const T& operator*() const { return _m; }
  T* operator->() { return &_m; }
  const T* operator->() const { return &_m; }

  inline bool is_dirty() const { return _dirty; }

  inline void make_clean() { _dirty = false; }
  inline void make_dirty() { _dirty = true; }

  /* Fn should return T and takes no arguments */
  template<typename Fn>
  inline void update( Fn&& f )
  {
    if ( _dirty )
    {
      _m = f();
      _dirty = false;
    }
  }

private:
  bool _dirty = true;
  T    _m;
}; /* dirty */

} /* util */

namespace aig
{

using aig_node = uint32_t;

enum class aig_type : uint8_t
{
  _const = 0u,
  _pi = 1u,
  _and = 2u,
  _latch = 3u
};

/******************************************************************************
 * aig_fuction                                                                *
 ******************************************************************************/

class aig_function
{
public:
  aig_function( aig_node node = 0u, bool complemented = false );

  explicit operator uint32_t() const;

  bool operator==( const aig_function& other ) const;
  bool operator!=( const aig_function& other ) const;
  bool operator<( const aig_function& other ) const;
  bool operator>( const aig_function& other ) const;

  aig_function operator!() const;
  aig_function operator^( bool value ) const;

public:
  union
  {
    uint32_t edge;
    struct
    {
      aig_node complemented : 1;
      aig_node node : 31;
    };
  };
}; /* aig_function */

aig_function::aig_function( aig_node node, bool complemented )
  : complemented( complemented )
  , node( node )
{
}

aig_function::operator uint32_t() const
{
  return edge;
}

bool aig_function::operator==( const aig_function& other ) const
{
  return edge == other.edge;
}

bool aig_function::operator!=( const aig_function& other ) const
{
  return !operator==( other );
}

bool aig_function::operator<( const aig_function& other ) const
{
  return edge < other.edge;
}

bool aig_function::operator>( const aig_function& other ) const
{
  return ( other < *this );
}

aig_function aig_function::operator!() const
{
  return aig_function( node, !complemented );
}

aig_function aig_function::operator^( bool value ) const
{
  return aig_function( node, complemented != value );
}

/******************************************************************************
 * aig_node_info                                                              *
 ******************************************************************************/

struct aig_node_info
{
public:
  aig_node_info( aig_type type = aig_type::_const );
  aig_node_info( const aig_function& left, const aig_function& right );
  aig_node_info( const aig_function& next, bool initial_value = false );

public:
  union
  {
    aig_function left;
    aig_function next;
  };
  aig_function right;
  union
  {
    uint32_t meta;
    struct
    {
      aig_type type       : 8;
      uint32_t is_output  : 1;
      uint32_t is_bad     : 1;
      uint32_t reset      : 1;
      uint32_t unused     : 21;
    };
  };
}; /* aig_node_info */

aig_node_info::aig_node_info( aig_type type )
  : type(type)
  , is_output(0u)
  , is_bad(0u)
  , unused(0u)
{
}

aig_node_info::aig_node_info( const aig_function& left, const aig_function& right )
  : left(left)
  , right(right)
  , type(aig_type::_and)
  , is_output(0u)
  , is_bad(0u)
  , unused(0u)
{
}

aig_node_info::aig_node_info( const aig_function& next, bool initial_value )
  : next(next)
  , right(0u)
  , type(aig_type::_latch)
  , is_output(0u)
  , is_bad(0u)
  , reset(uint32_t(initial_value))
  , unused(0u)
{
}

/******************************************************************************
 * aig_graph                                                                  *
 ******************************************************************************/

class aig_graph
{
public:
  using input_vec_t = std::vector<std::pair<aig_node,std::string>>;
  using latch_vec_t = std::vector<std::pair<aig_node,std::string>>;
  using output_vec_t = std::vector<std::pair<aig_function,std::string>>;
  using bad_state_vec_t = std::vector<std::pair<aig_function,std::string>>;
  using and_strash_key_t = std::tuple<aig_function, aig_function>;
  using node_vec_t = std::vector<aig_node>;
  using func_vec_t = std::vector<aig_function>;

  struct and_strash_hash : public std::unary_function<and_strash_key_t, std::size_t>
  {
    std::size_t operator()(const and_strash_key_t& k) const
    {
      auto seed = uint32_t(std::get<0>(k));
      seed ^= uint32_t(std::get<1>(k)) + 0x9e3779b9 + ( seed << 6 ) + ( seed >> 2 );
      return seed;
    }
  }; /* and_strash_hash */

  using and_strash_map_t = std::unordered_map<and_strash_key_t,aig_node,and_strash_hash>;

public:
  explicit aig_graph( const std::string& model_name = std::string() );

  /*** properties ***/
  void compute_input_nodes() const;
  void compute_latches_input_nodes() const;
  void compute_output_functions() const;
  void compute_bad_state_functions() const;
  void compute_latch_output_functions() const;
  void compute_parents() const;
  void compute_toporder() const;

  /*** create ***/
  aig_function get_constant( bool value = false ) const;
  aig_function create_pi( const std::string& name = std::string() );
  aig_function create_li( const std::string& name = std::string() );
  void create_po( const aig_function& f, const std::string& name );
  void create_bad_state( const aig_function& f, const std::string& name );
  void create_latch( const aig_function& curr, const aig_function& next, bool default_value );

  aig_function create_and( const aig_function& left, const aig_function& right );
  aig_function create_nand( const aig_function& left, const aig_function& right );
  aig_function create_or( const aig_function& left, const aig_function& right );
  aig_function create_nor( const aig_function& left, const aig_function& right );
  aig_function create_xor( const aig_function& left, const aig_function& right );
  aig_function create_ite( const aig_function& cond, const aig_function& t, const aig_function& e );
  aig_function create_implies( const aig_function& a, const aig_function& b );

  aig_function create_nary_and( const func_vec_t& v );
  aig_function create_nary_nand( const func_vec_t& v );
  aig_function create_nary_or( const func_vec_t& v );
  aig_function create_nary_nor( const func_vec_t& v );
  aig_function create_nary_xor( const func_vec_t& v );

  /*** access ***/
  input_vec_t& inputs();
  const input_vec_t& inputs() const;
  latch_vec_t& latches();
  const latch_vec_t& latches() const;
  output_vec_t& outputs();
  const output_vec_t& outputs() const;
  bad_state_vec_t& bad_states();
  const bad_state_vec_t& bad_states() const;

  const aig_node_info& operator[](aig_node node) const;
  aig_node_info& operator[](aig_node node);
  func_vec_t children(aig_node node) const;
  std::size_t size() const;

  /* replaces a with b */
  void substitute( const aig_function& a, const aig_function& b );

  /*** properties ***/
  const node_vec_t& input_nodes() const;
  const node_vec_t& latches_input_nodes() const;
  const func_vec_t& output_functions() const;
  const func_vec_t& bad_state_functions() const;
  const func_vec_t& latch_output_functions() const;
  const std::vector<node_vec_t>& parents() const;
  const node_vec_t& topological_nodes() const;

protected:
  aig_node _constant = 0u;
  std::string _model_name;
  std::vector<aig_node_info> _info;
  input_vec_t _inputs;
  latch_vec_t _latches;
  output_vec_t _outputs;
  bad_state_vec_t _bad_states;
  and_strash_map_t _and_strash;
  mutable util::dirty<node_vec_t>              _input_nodes;
  mutable util::dirty<node_vec_t>              _latches_input_nodes;
  mutable util::dirty<func_vec_t>              _output_functions;
  mutable util::dirty<func_vec_t>              _bad_state_functions;
  mutable util::dirty<func_vec_t>              _latch_output_functions;
  mutable util::dirty<node_vec_t>              _toporder;
  mutable util::dirty<std::vector<node_vec_t>> _parents;
}; /* aig_graph */

aig_graph::aig_graph( const std::string& model_name )
  : _model_name( model_name)
{
  assert( _constant == 0u );
  _info.push_back( aig_node_info() );
}

void aig_graph::compute_input_nodes() const
{
  /* no other properties required */
  if ( !_input_nodes.is_dirty() ) return;
  _input_nodes->clear();
  _input_nodes->reserve( _inputs.size() );
  std::transform( _inputs.begin(), _inputs.end(), std::back_inserter( *_input_nodes ),
                  [](const input_vec_t::value_type& v) { return v.first; } );
  assert( _input_nodes->size() == _inputs.size() );
  _input_nodes.make_clean();
}

void aig_graph::compute_latches_input_nodes() const
{
  /* no other properties required */
  if ( !_latches_input_nodes.is_dirty() ) return;
  _latches_input_nodes->clear();
  _latches_input_nodes->reserve( _latches.size() );
  std::transform( _latches.begin(), _latches.end(), std::back_inserter( *_latches_input_nodes ),
                  [](const latch_vec_t::value_type& v) { return v.first; } );
  assert( _latches_input_nodes->size() == _latches.size() );
  _latches_input_nodes.make_clean();
}

void aig_graph::compute_output_functions() const
{
  /* no other properties required */
  if ( !_output_functions.is_dirty() ) return;
  _output_functions->clear();
  _output_functions->reserve( _outputs.size() );
  std::transform( _outputs.begin(), _outputs.end(), std::back_inserter( *_output_functions ),
                  [](const output_vec_t::value_type& v) { return v.first; } );
  assert( _output_functions->size() == _outputs.size() );
  _output_functions.make_clean();
}

void aig_graph::compute_bad_state_functions() const
{
  /* no other properties required */
  if ( !_bad_state_functions.is_dirty() ) return;
  _bad_state_functions->clear();
  _bad_state_functions->reserve( _bad_states.size() );
  std::transform( _bad_states.begin(), _bad_states.end(), std::back_inserter( *_bad_state_functions ),
                  [](const bad_state_vec_t::value_type& v) { return v.first; } );
  assert( _bad_state_functions->size() == _bad_states.size() );
  _bad_state_functions.make_clean();
}

void aig_graph::compute_latch_output_functions() const
{
  /* no other properties required */
  if ( !_latch_output_functions.is_dirty() ) return;
  _latch_output_functions->clear();
  _latch_output_functions->reserve( _latches.size() );
  for ( const auto& elem : _latches )
  {
    _latch_output_functions->push_back( _info[elem.first].next );
  }
  assert( _latch_output_functions->size() == _latch_output_functions->size() );
  _latch_output_functions.make_clean();
}

void aig_graph::compute_parents() const
{
  /* no other properties required */
  if ( !_parents.is_dirty() ) return;
  _parents->clear();
  _parents->resize( _info.size() );

  for ( auto i = 0u; i < _info.size(); ++i )
  {
    const auto info = _info[i];
    if ( info.type != aig_type::_and ) continue;
    aig_node children_nodes[] = { info.left.node, info.right.node };
    for ( auto j = 0u; j < 2u; ++j )
    {
      auto& parents = (*_parents)[ children_nodes[j] ];
      if ( std::find( parents.begin(), parents.end(), i ) == parents.end() )
      {
        parents.push_back( i );
      }
    }
  }

  _parents.make_clean();
}

void aig_graph::compute_toporder() const
{
  /* requires: fanout, parents */
  compute_parents();
  if ( !_toporder.is_dirty() ) return;
  
  std::vector<uint32_t> fanout;
  fanout.resize( _info.size(), 2u );
  fanout[_constant] = 0u;
  for( const auto& i : _inputs )
  {
    fanout[i.first] = 0u;
  }
  for( const auto& l : _latches )
  {
    fanout[l.first] = 0u;
  }

  std::list<aig_node> S;
  S.push_back( _constant );

  for ( const auto& i : _inputs )
  {
    S.push_back( i.first );
  }
  for ( const auto& l : _latches )
  {
    S.push_back( l.first );
  }

  _toporder->clear();
  while ( !S.empty() )
  {
    auto n = S.back();
    S.pop_back();

    _toporder->push_back( n );

    for ( auto m : (*_parents)[n] )
    {
      --fanout[m];
      if( !fanout[m] )
      {
        S.push_back(m);
      }
    }
  }

  _toporder.make_clean();
}

const std::vector<aig_node>& aig_graph::topological_nodes() const
{
  compute_toporder();
  return *_toporder;
}

const std::vector<std::vector<aig_node>>& aig_graph::parents() const
{
  compute_parents();
  return *_parents;
}

const aig_graph::node_vec_t& aig_graph::input_nodes() const
{
  compute_input_nodes();
  return *_input_nodes;
}

const aig_graph::node_vec_t& aig_graph::latches_input_nodes() const
{
  compute_latches_input_nodes();
  return *_latches_input_nodes;
}

const aig_graph::func_vec_t& aig_graph::output_functions() const
{
  compute_output_functions();
  return *_output_functions;
}

const aig_graph::func_vec_t& aig_graph::bad_state_functions() const
{
  compute_bad_state_functions();
  return *_bad_state_functions;
}

const aig_graph::func_vec_t& aig_graph::latch_output_functions() const
{
  compute_latch_output_functions();
  return *_latch_output_functions;
}

aig_function aig_graph::get_constant( bool value ) const
{
  return aig_function( _constant, value );
}

aig_function aig_graph::create_pi( const std::string& name )
{
  _input_nodes.make_dirty();
  const auto node = _info.size();
  _info.push_back( aig_node_info(aig_type::_pi) );
  _inputs.push_back( {node, name} );
  return aig_function( node );
}

aig_function aig_graph::create_li( const std::string& name )
{
  _latches_input_nodes.make_dirty();
  const auto node = _info.size();
  _info.emplace_back( aig_function(0, false), false );
  _latches.emplace_back( node, name );
  return aig_function( node );
}

void aig_graph::create_po( const aig_function& f, const std::string& name )
{
  _output_functions.make_dirty();
  _outputs.push_back( {f, name} );
  _info[f.node].is_output = 1;
}

void aig_graph::create_bad_state( const aig_function& f, const std::string& name )
{
  _bad_state_functions.make_dirty();
  _bad_states.push_back( {f, name} );
  _info[f.node].is_bad = 1;
}

void aig_graph::create_latch( const aig_function& curr, const aig_function& next, bool default_value )
{
  _info[curr.node].next = next;
  _info[curr.node].reset = default_value;
}

aig_function aig_graph::create_and( const aig_function& left, const aig_function& right )
{
  /* special cases */
  if ( left == get_constant(false) )  { return get_constant( false ); }
  if ( right == get_constant(false) ) { return get_constant( false ); }
  if ( left == get_constant(true) )   { return right; }
  if ( right == get_constant(true) )  { return left; }
  if ( left == right  )               { return left; }
  if ( left == !right )               { return get_constant( false ); }

  /* structural hashing */
  const auto key = (left < right) ? std::make_tuple( left, right ) : std::make_tuple( right, left );
  const auto it = _and_strash.find( key );
  if ( it != _and_strash.end() )
  {
    const auto& f = aig_function( it->second );
    return f;
  }

  /* insert node */
  _toporder.make_dirty();
  const auto node = _info.size();
  _info.push_back( aig_node_info( std::get<0>(key), std::get<1>(key) ) );
  _and_strash[key] = node;

  /* maintain parent property: node is a parent of left and right */
  if ( !_parents.is_dirty() )
  {
    (*_parents).resize( (*_parents).size() + 1u );
    auto& left_parents = (*_parents)[ left.node ];
    auto& right_parents = (*_parents)[ right.node ];
    if ( std::find( left_parents.begin(), left_parents.end(), node ) == left_parents.end() )
    {
      left_parents.push_back( node );
    }
    if ( std::find( right_parents.begin(), right_parents.end(), node ) == right_parents.end() )
    {
      right_parents.push_back( node );
    }
  }

  return aig_function( node );
}

aig_function aig_graph::create_nand( const aig_function& left, const aig_function& right )
{
  return !create_and( left, right );
}

aig_function aig_graph::create_or( const aig_function& left, const aig_function& right )
{
  return !create_and( !left, !right );
}

aig_function aig_graph::create_nor( const aig_function& left, const aig_function& right )
{
  return create_and( !left, !right );
}

aig_function aig_graph::create_xor( const aig_function& left, const aig_function& right )
{
  return !create_or( create_and( left, right), create_and( !left, !right ) );
}

aig_function aig_graph::create_ite( const aig_function& cond, const aig_function& t, const aig_function& e )
{
  return create_or( create_and( cond, t ), create_and( !cond, e ) );
}

aig_function aig_graph::create_implies( const aig_function& a, const aig_function& b )
{
  return create_or( !a, b );
}

aig_function aig_graph::create_nary_and( const std::vector< aig_function >& v )
{
  const auto size = v.size();
  assert( size >= 1u );

  if ( size == 1u )
  {
    return v[0];
  }

  aig_function result;
  if ( size >= 2u )
  {
    result = create_and( v[0], v[1] );
  }
  for ( unsigned u = 2u; u < size; ++u )
  {
    result = create_and( result, v[u] );
  }
  return result;
}

aig_function aig_graph::create_nary_nand( const std::vector< aig_function >& v )
{
  return !create_nary_and( v );
}

aig_function aig_graph::create_nary_or( const std::vector< aig_function >& v )
{
  const auto size = v.size();
  assert( size >= 1u );

  if ( size == 1u )
  {
    return v[0];
  }

  aig_function result;
  if ( size >= 2u )
  {
    result = create_or( v[0], v[1] );
  }
  for ( unsigned u = 2u; u < size; ++u )
  {
    result = create_or( result, v[u] );
  }
  return result;
}

aig_function aig_graph::create_nary_nor( const std::vector< aig_function >& v )
{
  return !create_nary_or( v );
}

aig_function aig_graph::create_nary_xor( const std::vector< aig_function >& v )
{
  const auto size = v.size();
  assert( size >= 1u );

  if ( size == 1u )
  {
    return v[0];
  }

  aig_function result;
  if ( size >= 2u )
  {
    result = create_xor( v[0], v[1] );
  }
  for ( unsigned u = 2u; u < size; ++u )
  {
    result = create_xor( result, v[u] );
  }
  return result;
}

aig_graph::input_vec_t& aig_graph::inputs()
{
  return _inputs;
}

const aig_graph::input_vec_t& aig_graph::inputs() const
{
  return _inputs;
}

aig_graph::latch_vec_t& aig_graph::latches()
{
  return _latches;
}

const aig_graph::latch_vec_t& aig_graph::latches() const
{
  return _latches;
}

aig_graph::output_vec_t& aig_graph::outputs()
{
  return _outputs;
}

const aig_graph::output_vec_t& aig_graph::outputs() const
{
  return _outputs;
}

aig_graph::bad_state_vec_t& aig_graph::bad_states()
{
  return _bad_states;
}

const aig_graph::bad_state_vec_t& aig_graph::bad_states() const
{
  return _bad_states;
}

const aig_node_info& aig_graph::operator[](aig_node node) const
{
  return _info[node];
}

aig_node_info& aig_graph::operator[](aig_node node)
{
  return _info[node];
}

std::vector<aig_function> aig_graph::children(aig_node node) const
{
  if ( _info[node].type == aig_type::_and )
  {
    return { _info[node].left, _info[node].right };
  }
  else if ( _info[node].type == aig_type::_latch )
  {
    return { _info[node].next };
  }
  else
  {
    return {};
  }
}

std::size_t aig_graph::size() const
{
  return _info.size();
}

void aig_graph::substitute( const aig_function& a, const aig_function& b )
{
  /* requires: parents */
  compute_parents();

  /* make a copy of the parents of a */
  const auto old_parents = (*_parents)[ a.node ];

  for ( const auto& parent_node : old_parents )
  {
    auto& info = _info[ parent_node ];

    /* parent_node has to be an AND node because other nodes do not have children */
    assert( info.type == aig_type::_and );

    /* either left or right or both have to be updated with b */
    if ( a.node == info.left.node )  { info.left.node  = b.node; info.left.complemented  ^= (a.complemented ^ b.complemented); }
    if ( a.node == info.right.node ) { info.right.node = b.node; info.right.complemented ^= (a.complemented ^ b.complemented); }

    /*** maintain parent property ***/

    /* get a reference to the parents of a.node (within the graph) */
    auto& current_parents = (*_parents)[ a.node ];

    /* remove parent_node from its parents */
    current_parents.erase( std::remove( current_parents.begin(), current_parents.end(), parent_node ), current_parents.end() );

    /* add parent_node to the parents of b.node */
    auto& new_parents = (*_parents)[ b.node ];
    if ( std::find( new_parents.begin(), new_parents.end(), parent_node ) == new_parents.end() )
    {
      new_parents.push_back( parent_node );
    }
  }

  /* invalidates: toporder */
  _toporder.make_dirty();
}

} // namespace aig
