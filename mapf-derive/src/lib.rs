/*
 * Copyright (C) 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput, Fields};

#[proc_macro_derive(
    Domain,
    attributes(
        domain,
        activity,
        weight,
        heuristic,
        closer,
        satisfier,
        initializer,
        connector,
        arrival_keyring
    )
)]
pub fn derive_domain(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let mut state_type = None;
    let mut error_type = None;

    for attr in &input.attrs {
        if attr.path().is_ident("domain") {
            let _ = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("state") {
                    let value = meta.value()?;
                    state_type = Some(value.parse::<syn::Type>()?);
                    Ok(())
                } else if meta.path.is_ident("error") {
                    let value = meta.value()?;
                    error_type = Some(value.parse::<syn::Type>()?);
                    Ok(())
                } else {
                    Err(meta.error("unsupported domain attribute"))
                }
            });
        }
    }

    let state_type =
        state_type.expect("Domain derive requires a 'state' attribute: #[domain(state = ...)]");
    let error_type = error_type.unwrap_or_else(|| syn::parse_quote!(anyhow::Error));

    let mut expanded = quote! {};

    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    expanded.extend(quote! {
        impl #impl_generics ::mapf::domain::Domain for #name #ty_generics #where_clause {
            type State = #state_type;
            type Error = #error_type;
        }
    });

    if let Data::Struct(data) = &input.data {
        if let Fields::Named(fields) = &data.fields {
            for field in &fields.named {
                let field_name = &field.ident;
                let field_ty = &field.ty;
                for attr in &field.attrs {
                    if attr.path().is_ident("activity") {
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Activity<#state_type> for #name #ty_generics #where_clause {
                                type Action = <#field_ty as ::mapf::domain::Activity<#state_type>>::Action;
                                type ActivityError = <#field_ty as ::mapf::domain::Activity<#state_type>>::ActivityError;
                                type Choices<'a> = <#field_ty as ::mapf::domain::Activity<#state_type>>::Choices<'a>
                                where
                                    Self: 'a,
                                    Self::Action: 'a,
                                    Self::ActivityError: 'a,
                                    #state_type: 'a;

                                fn choices<'a>(&'a self, from_state: #state_type) -> Self::Choices<'a>
                                where
                                    Self: 'a,
                                    Self::Action: 'a,
                                    Self::ActivityError: 'a,
                                    #state_type: 'a
                                {
                                    self.#field_name.choices(from_state)
                                }
                            }
                        });
                    } else if attr.path().is_ident("weight") {
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Weight<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action> for #name #ty_generics #where_clause {
                                type Cost = <#field_ty as ::mapf::domain::Weight<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action>>::Cost;
                                type WeightError = <#field_ty as ::mapf::domain::Weight<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action>>::WeightError;
                                fn cost(&self, from_state: &#state_type, action: &<Self as ::mapf::domain::Activity<#state_type>>::Action, to_state: &#state_type) -> Result<Option<Self::Cost>, Self::WeightError> {
                                    self.#field_name.cost(from_state, action, to_state)
                                }
                                fn initial_cost(&self, for_state: &#state_type) -> Result<Option<Self::Cost>, Self::WeightError> {
                                    self.#field_name.initial_cost(for_state)
                                }
                            }
                        });
                    } else if attr.path().is_ident("heuristic") {
                        // Assuming Goal is #state_type by default
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Heuristic<#state_type, #state_type> for #name #ty_generics #where_clause {
                                type CostEstimate = <#field_ty as ::mapf::domain::Heuristic<#state_type, #state_type>>::CostEstimate;
                                type HeuristicError = <#field_ty as ::mapf::domain::Heuristic<#state_type, #state_type>>::HeuristicError;
                                fn estimate_remaining_cost(&self, from_state: &#state_type, to_goal: &#state_type) -> Result<Option<Self::CostEstimate>, Self::HeuristicError> {
                                    self.#field_name.estimate_remaining_cost(from_state, to_goal)
                                }
                            }
                        });
                    } else if attr.path().is_ident("closer") {
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Closable<#state_type> for #name #ty_generics #where_clause {
                                type ClosedSet<T> = <#field_ty as ::mapf::domain::Closable<#state_type>>::ClosedSet<T>;
                                fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
                                    self.#field_name.new_closed_set()
                                }
                            }
                        });
                    } else if attr.path().is_ident("satisfier") {
                        // Assuming Goal is #state_type
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Satisfiable<#state_type, #state_type> for #name #ty_generics #where_clause {
                                type SatisfactionError = <#field_ty as ::mapf::domain::Satisfiable<#state_type, #state_type>>::SatisfactionError;
                                fn is_satisfied(&self, by_state: &#state_type, for_goal: &#state_type) -> Result<bool, Self::SatisfactionError> {
                                    self.#field_name.is_satisfied(by_state, for_goal)
                                }
                            }
                        });
                    } else if attr.path().is_ident("initializer") {
                        // Assuming Start and Goal are #state_type
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Initializable<#state_type, #state_type, #state_type> for #name #ty_generics #where_clause {
                                type InitialError = <#field_ty as ::mapf::domain::Initializable<#state_type, #state_type, #state_type>>::InitialError;
                                type InitialStates<'a> = <#field_ty as ::mapf::domain::Initializable<#state_type, #state_type, #state_type>>::InitialStates<'a>
                                where
                                    Self: 'a,
                                    Self::InitialError: 'a,
                                    #state_type: 'a;

                                fn initialize<'a>(&'a self, from_start: #state_type, to_goal: &#state_type) -> Self::InitialStates<'a>
                                where
                                    Self: 'a,
                                    Self::InitialError: 'a,
                                    #state_type: 'a
                                {
                                    self.#field_name.initialize(from_start, to_goal)
                                }
                            }
                        });
                    } else if attr.path().is_ident("connector") {
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::Connectable<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action, #state_type> for #name #ty_generics #where_clause {
                                type ConnectionError = <#field_ty as ::mapf::domain::Connectable<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action, #state_type>>::ConnectionError;
                                type Connections<'a> = <#field_ty as ::mapf::domain::Connectable<#state_type, <Self as ::mapf::domain::Activity<#state_type>>::Action, #state_type>>::Connections<'a>
                                where
                                    Self: 'a,
                                    Self::ConnectionError: 'a,
                                    #state_type: 'a,
                                    <Self as ::mapf::domain::Activity<#state_type>>::Action: 'a;

                                fn connect<'a>(&'a self, from_state: #state_type, to_target: &'a #state_type) -> Self::Connections<'a>
                                where
                                    Self: 'a,
                                    Self::ConnectionError: 'a,
                                    #state_type: 'a,
                                    <Self as ::mapf::domain::Activity<#state_type>>::Action: 'a
                                {
                                    self.#field_name.connect(from_state, to_target)
                                }
                            }
                        });
                    } else if attr.path().is_ident("arrival_keyring") {
                        expanded.extend(quote! {
                            impl #impl_generics ::mapf::domain::ArrivalKeyring<<#field_ty as ::mapf::domain::Keyed>::Key, #state_type, #state_type> for #name #ty_generics #where_clause {
                                type ArrivalKeyError = <#field_ty as ::mapf::domain::ArrivalKeyring<<#field_ty as ::mapf::domain::Keyed>::Key, #state_type, #state_type>>::ArrivalKeyError;
                                type ArrivalKeys<'a> = <#field_ty as ::mapf::domain::ArrivalKeyring<<#field_ty as ::mapf::domain::Keyed>::Key, #state_type, #state_type>>::ArrivalKeys<'a>
                                where
                                    Self: 'a,
                                    Self::ArrivalKeyError: 'a,
                                    <#field_ty as ::mapf::domain::Keyed>::Key: 'a,
                                    #state_type: 'a;

                                fn get_arrival_keys<'a>(&'a self, start: &#state_type, goal: &#state_type) -> Self::ArrivalKeys<'a>
                                where
                                    Self: 'a,
                                    Self::ArrivalKeyError: 'a,
                                    <#field_ty as ::mapf::domain::Keyed>::Key: 'a,
                                    #state_type: 'a
                                {
                                    self.#field_name.get_arrival_keys(start, goal)
                                }
                            }
                        });
                    }
                }
            }
        }
    }

    TokenStream::from(expanded)
}
